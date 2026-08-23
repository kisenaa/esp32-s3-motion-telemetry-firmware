#include "sensor_task.h"

#include "app_state.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mma8451q.h"
#include "sdkconfig.h"

static const char *TAG = "sensor_task";

static QueueHandle_t s_sample_queue = nullptr;
static TaskHandle_t s_task_handle = nullptr;
static bool s_gpio_handler_registered = false;

static constexpr gpio_num_t SENSOR_INT_GPIO =
    (gpio_num_t)CONFIG_APP_MMA8451Q_INT1_GPIO;

static constexpr TickType_t INTERRUPT_TIMEOUT =
    pdMS_TO_TICKS(CONFIG_APP_SENSOR_INTERRUPT_TIMEOUT_MS);

static_assert(CONFIG_APP_MMA8451Q_INT1_GPIO >= 0,
              "MMA8451Q INT1 GPIO must be valid");

static_assert(CONFIG_APP_MMA8451Q_INT1_GPIO < 64,
              "MMA8451Q INT1 GPIO must fit the GPIO pin mask");

static_assert(INTERRUPT_TIMEOUT > 0,
              "Interrupt timeout must produce at least one RTOS tick");

static void IRAM_ATTR sensor_task_gpio_isr(void *argument) {
  (void)argument;

  BaseType_t higher_priority_task_woken = pdFALSE;

  vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void sensor_task_release_gpio(void) {
  (void)gpio_intr_disable(SENSOR_INT_GPIO);

  if (s_gpio_handler_registered) {
    const esp_err_t err = gpio_isr_handler_remove(SENSOR_INT_GPIO);

    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to remove INT1 ISR handler: %s",
               esp_err_to_name(err));
    }

    s_gpio_handler_registered = false;
  }

  const esp_err_t err = gpio_reset_pin(SENSOR_INT_GPIO);

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to reset INT1 GPIO: %s", esp_err_to_name(err));
  }
}

static esp_err_t sensor_task_configure_gpio(void) {
  const gpio_config_t config = {
      .pin_bit_mask = 1ULL << CONFIG_APP_MMA8451Q_INT1_GPIO,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t err = gpio_config(&config);

  if (err != ESP_OK) {
    return err;
  }

  err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    (void)gpio_reset_pin(SENSOR_INT_GPIO);
    return err;
  }

  err = gpio_isr_handler_add(SENSOR_INT_GPIO, sensor_task_gpio_isr, nullptr);

  if (err != ESP_OK) {
    (void)gpio_reset_pin(SENSOR_INT_GPIO);
    return err;
  }

  s_gpio_handler_registered = true;

  err = gpio_set_intr_type(SENSOR_INT_GPIO, GPIO_INTR_NEGEDGE);

  if (err != ESP_OK) {
    sensor_task_release_gpio();
    return err;
  }

  err = gpio_intr_enable(SENSOR_INT_GPIO);

  if (err != ESP_OK) {
    sensor_task_release_gpio();
    return err;
  }

  ESP_LOGI(TAG, "MMA8451Q INT1 configured on GPIO%d",
           CONFIG_APP_MMA8451Q_INT1_GPIO);

  return ESP_OK;
}

static void sensor_task_handle_read_error(esp_err_t err) {
  (void)app_state_record_i2c_error();
  ESP_LOGE(TAG, "MMA8451Q sample read failed: %s", esp_err_to_name(err));
}

static void sensor_task_publish(const app_sample_t *sample) {
  if (xQueueSend(s_sample_queue, sample, 0) == pdTRUE) {
    return;
  }

  (void)app_state_record_queue_drop();
  ESP_LOGW(TAG, "Sample queue full; newest sample dropped");
}

static void sensor_task_acquire(uint32_t interrupt_count) {
  if (interrupt_count > 0U) {
    ESP_LOGD(TAG, "Acquiring sample after %u interrupt(s)", interrupt_count);
    (void)app_state_record_sensor_interrupts(interrupt_count);
  } else if (gpio_get_level(SENSOR_INT_GPIO) != 0) {
    return;
  } else {
    ESP_LOGW(TAG, "INT1 remained active without a new GPIO edge");
  }

  uint8_t sources = 0;
  esp_err_t err = mma8451q_read_interrupt_source(&sources);

  if (err != ESP_OK) {
    sensor_task_handle_read_error(err);
  } else if ((sources & MMA8451Q_INTERRUPT_DATA_READY) == 0U) {
    ESP_LOGW(TAG, "INT1 asserted without DATA_READY");
    return;
  }

  mma8451q_sample_t acceleration;
  err = mma8451q_read_sample(&acceleration);

  if (err != ESP_OK) {
    sensor_task_handle_read_error(err);
    return;
  }

  const app_sample_t sample = {
      .timestamp_us = esp_timer_get_time(),
      .acceleration = acceleration,
  };

  (void)app_state_record_sample(&sample);
  sensor_task_publish(&sample);
}

static void sensor_task_run(void *argument) {
  (void)argument;

  while (true) {
    const uint32_t interrupt_count =
        ulTaskNotifyTake(pdTRUE, INTERRUPT_TIMEOUT);

    sensor_task_acquire(interrupt_count);
  }
}

static void sensor_task_rollback_start(void) {
  sensor_task_release_gpio();

  if (s_task_handle != nullptr) {
    vTaskDelete(s_task_handle);
    s_task_handle = nullptr;
  }

  s_sample_queue = nullptr;
}

esp_err_t sensor_task_start(QueueHandle_t sample_queue,
                            TaskHandle_t *task_handle) {
  if (sample_queue == nullptr || task_handle == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_task_handle != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  s_sample_queue = sample_queue;

  const BaseType_t result = xTaskCreate(
      sensor_task_run, "sensor_task", CONFIG_APP_SENSOR_TASK_STACK_SIZE,
      nullptr, CONFIG_APP_SENSOR_TASK_PRIORITY, &s_task_handle);

  if (result != pdPASS) {
    s_sample_queue = nullptr;
    s_task_handle = nullptr;
    return ESP_ERR_NO_MEM;
  }

  esp_err_t err = sensor_task_configure_gpio();

  if (err != ESP_OK) {
    sensor_task_rollback_start();
    return err;
  }

  err = mma8451q_enable_data_ready_interrupt(true);

  if (err != ESP_OK) {
    sensor_task_rollback_start();
    return err;
  }

  *task_handle = s_task_handle;

  ESP_LOGI(TAG, "Interrupt-driven sensor task started");
  return ESP_OK;
}

void sensor_task_stop(void) {
  if (s_task_handle == nullptr) {
    return;
  }

  (void)gpio_intr_disable(SENSOR_INT_GPIO);

  const esp_err_t err = mma8451q_enable_data_ready_interrupt(false);

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to disable DATA_READY: %s", esp_err_to_name(err));
  }

  sensor_task_release_gpio();

  vTaskDelete(s_task_handle);
  s_task_handle = nullptr;
  s_sample_queue = nullptr;

  ESP_LOGI(TAG, "Sensor task stopped");
}
