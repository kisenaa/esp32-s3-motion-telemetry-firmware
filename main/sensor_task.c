#include "sensor_task.h"
#include "app_state.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "mma8451q.h"
#include "sdkconfig.h"
#include "status_led.h"
#include <inttypes.h>

static const char *const TAG = "sensor_task";

static QueueHandle_t s_sample_queue = nullptr;
static TaskHandle_t s_task_handle = nullptr;
static StaticSemaphore_t s_sensor_mutex_storage;
static SemaphoreHandle_t s_sensor_mutex = nullptr;
static bool s_gpio_handler_registered = false;
static volatile bool s_stop_requested = false;
static TaskHandle_t s_stop_waiter = nullptr;

static constexpr gpio_num_t SENSOR_INT_GPIO =
    (gpio_num_t)CONFIG_APP_MMA8451Q_INT1_GPIO;

static constexpr TickType_t INTERRUPT_TIMEOUT =
    pdMS_TO_TICKS(CONFIG_APP_SENSOR_INTERRUPT_TIMEOUT_MS);

static constexpr TickType_t RECOVERY_INTERVAL =
    pdMS_TO_TICKS(CONFIG_APP_SENSOR_RECOVERY_INTERVAL_MS);

static constexpr uint32_t ODR_MILLIHZ[] = {
    800000U, 400000U, 200000U, 100000U, 50000U, 12500U, 6250U, 1560U,
};

static uint32_t s_consecutive_failures = 0U;
static TickType_t s_next_recovery_time = 0U;

static mma8451q_odr_t s_current_odr =
    (mma8451q_odr_t)CONFIG_APP_MMA8451Q_DEFAULT_ODR;

static_assert(CONFIG_APP_MMA8451Q_INT1_GPIO >= 0,
              "MMA8451Q INT1 GPIO must be valid");

static_assert(CONFIG_APP_MMA8451Q_INT1_GPIO < 64,
              "MMA8451Q INT1 GPIO must fit the GPIO pin mask");

static_assert(INTERRUPT_TIMEOUT > 0,
              "Interrupt timeout must produce at least one RTOS tick");

static_assert(RECOVERY_INTERVAL > 0,
              "Recovery interval must produce at least one RTOS tick");

static_assert(CONFIG_APP_SENSOR_FAILURE_THRESHOLD > 0,
              "Sensor failure threshold must be positive");

static_assert(sizeof(ODR_MILLIHZ) / sizeof(ODR_MILLIHZ[0]) == 8U,
              "Every MMA8451Q ODR must have a millihertz value");

static esp_err_t sensor_task_lock(void) {
  if (s_sensor_mutex == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  return xSemaphoreTake(s_sensor_mutex, portMAX_DELAY) == pdTRUE ? ESP_OK
                                                                 : ESP_FAIL;
}

static void sensor_task_unlock(void) { xSemaphoreGive(s_sensor_mutex); }

static void IRAM_ATTR sensor_task_gpio_isr(void *argument) {
  (void)argument;

  BaseType_t higher_priority_task_woken = pdFALSE;

  vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static uint32_t sensor_task_odr_millihz(mma8451q_odr_t odr) {
  const size_t index = (size_t)odr;

  if (index >= sizeof(ODR_MILLIHZ) / sizeof(ODR_MILLIHZ[0])) {
    return 0U;
  }

  return ODR_MILLIHZ[index];
}

static void sensor_task_restore_runtime_led(void) {
  app_state_snapshot_t state;

  if (app_state_get_snapshot(&state) != ESP_OK) {
    return;
  }

  const status_led_state_t led_state =
      !state.sensor_ready || !state.storage_ready ? STATUS_LED_ERROR
      : state.logging_enabled                     ? STATUS_LED_LOGGING
                                                  : STATUS_LED_READY;

  (void)status_led_set_state(led_state);
}

static esp_err_t sensor_task_reinitialize(void) {
  (void)gpio_intr_disable(SENSOR_INT_GPIO);
  (void)app_state_record_sensor_reinitialization();

  esp_err_t err = sensor_task_lock();

  if (err != ESP_OK) {
    return err;
  }

  const esp_err_t deinit_err = mma8451q_deinit();

  if (deinit_err != ESP_OK) {
    ESP_LOGW(TAG, "Sensor cleanup reported: %s", esp_err_to_name(deinit_err));
    sensor_task_unlock();
    return deinit_err;
  }

  err = mma8451q_init();

  if (err == ESP_OK) {
    err = mma8451q_set_odr(s_current_odr);
  }

  if (err == ESP_OK) {
    err = mma8451q_enable_data_ready_interrupt(true);
  }

  sensor_task_unlock();

  if (err != ESP_OK) {
    return err;
  }

  (void)ulTaskNotifyTake(pdTRUE, 0);
  if (s_stop_requested) {
    return ESP_ERR_INVALID_STATE;
  }

  return gpio_intr_enable(SENSOR_INT_GPIO);
}

static void sensor_task_attempt_recovery(void) {
  const TickType_t now = xTaskGetTickCount();

  if (s_next_recovery_time != 0U && (int32_t)(now - s_next_recovery_time) < 0) {
    return;
  }

  ESP_LOGW(TAG, "Attempting MMA8451Q recovery");

  const esp_err_t err = sensor_task_reinitialize();

  if (err != ESP_OK) {
    s_next_recovery_time = now + RECOVERY_INTERVAL;
    ESP_LOGE(TAG, "MMA8451Q recovery failed: %s", esp_err_to_name(err));
    return;
  }

  s_consecutive_failures = 0U;
  s_next_recovery_time = 0U;

  (void)app_state_set_sensor(true, sensor_task_odr_millihz(s_current_odr));

  sensor_task_restore_runtime_led();
  ESP_LOGI(TAG, "MMA8451Q recovery succeeded");
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

  ESP_LOGI(TAG, "MMA8451Q INT1 configured on GPIO%d",
           CONFIG_APP_MMA8451Q_INT1_GPIO);

  return ESP_OK;
}

static void sensor_task_handle_read_error(esp_err_t err) {
  (void)app_state_record_i2c_error();

  if (s_consecutive_failures < UINT32_MAX) {
    ++s_consecutive_failures;
  }

  ESP_LOGW(TAG, "MMA8451Q operation failed (%" PRIu32 "/%d): %s",
           s_consecutive_failures, CONFIG_APP_SENSOR_FAILURE_THRESHOLD,
           esp_err_to_name(err));

  if (s_consecutive_failures < CONFIG_APP_SENSOR_FAILURE_THRESHOLD) {
    return;
  }

  (void)app_state_set_sensor(false, 0U);
  (void)status_led_set_state(STATUS_LED_ERROR);
  sensor_task_attempt_recovery();
}

static void sensor_task_record_success(void) {
  s_consecutive_failures = 0U;
  s_next_recovery_time = 0U;
}

static void sensor_task_publish(const app_sample_t *sample) {
  if (xQueueSend(s_sample_queue, sample, 0) == pdTRUE) {
    return;
  }

  (void)app_state_record_queue_drop();
  ESP_LOGW(TAG, "Sample queue full; newest sample dropped");
}

static esp_err_t
sensor_task_read_acceleration(mma8451q_sample_t *acceleration) {
  esp_err_t err = sensor_task_lock();

  if (err != ESP_OK) {
    return err;
  }

  uint8_t sources = 0;
  err = mma8451q_read_interrupt_source(&sources);

  if (err != ESP_OK) {
    sensor_task_unlock();
    return err;
  }

  if ((sources & MMA8451Q_INTERRUPT_DATA_READY) == 0U) {
    sensor_task_unlock();
    return ESP_ERR_NOT_FOUND;
  }

  const esp_err_t sample_err = mma8451q_read_sample(acceleration);

  sensor_task_unlock();
  if (sample_err != ESP_OK) {
    return sample_err;
  }

  return ESP_OK;
}

static void sensor_task_check_connection(void) {
  esp_err_t err = sensor_task_lock();

  if (err == ESP_OK) {
    err = mma8451q_check_connection();
    sensor_task_unlock();
  }

  if (err == ESP_OK) {
    sensor_task_record_success();
    return;
  }

  sensor_task_handle_read_error(err);
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

  mma8451q_sample_t acceleration;
  const esp_err_t err = sensor_task_read_acceleration(&acceleration);

  if (err == ESP_ERR_NOT_FOUND) {
    ESP_LOGW(TAG, "INT1 asserted without DATA_READY");
    return;
  }

  if (err != ESP_OK) {
    sensor_task_handle_read_error(err);
    return;
  }

  sensor_task_record_success();

  const app_sample_t sample = {
      .timestamp_us = esp_timer_get_time(),
      .acceleration = acceleration,
  };

  (void)app_state_record_sample(&sample);
  sensor_task_publish(&sample);
}

static void sensor_task_run(void *argument) {
  (void)argument;

  while (!s_stop_requested) {
    const uint32_t interrupt_count =
        ulTaskNotifyTake(pdTRUE, INTERRUPT_TIMEOUT);

    if (interrupt_count > 0U || gpio_get_level(SENSOR_INT_GPIO) == 0) {
      sensor_task_acquire(interrupt_count);
      continue;
    }

    if (s_consecutive_failures >= CONFIG_APP_SENSOR_FAILURE_THRESHOLD) {
      sensor_task_attempt_recovery();
      continue;
    }

    sensor_task_check_connection();
  }

  if (s_stop_waiter != nullptr) {
    xTaskNotifyGive(s_stop_waiter);
  }
}

static void sensor_task_rollback_start(void) { sensor_task_stop(); }

esp_err_t sensor_task_start(QueueHandle_t sample_queue,
                            TaskHandle_t *task_handle) {
  if (sample_queue == nullptr || task_handle == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_task_handle != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  s_sample_queue = sample_queue;
  s_sensor_mutex = xSemaphoreCreateMutexStatic(&s_sensor_mutex_storage);

  if (s_sensor_mutex == nullptr) {
    s_sample_queue = nullptr;
    return ESP_ERR_NO_MEM;
  }

  s_current_odr = (mma8451q_odr_t)CONFIG_APP_MMA8451Q_DEFAULT_ODR;
  s_consecutive_failures = 0U;
  s_next_recovery_time = 0U;
  s_stop_requested = false;
  s_stop_waiter = nullptr;

  esp_err_t err = sensor_task_configure_gpio();

  if (err != ESP_OK) {
    vSemaphoreDelete(s_sensor_mutex);
    s_sensor_mutex = nullptr;
    s_sample_queue = nullptr;
    return err;
  }

  err = sensor_task_lock();

  if (err == ESP_OK) {
    err = mma8451q_enable_data_ready_interrupt(true);
    sensor_task_unlock();
  }

  if (err != ESP_OK) {
    sensor_task_release_gpio();
    vSemaphoreDelete(s_sensor_mutex);
    s_sensor_mutex = nullptr;
    s_sample_queue = nullptr;
    return err;
  }

  const BaseType_t result = xTaskCreate(
      sensor_task_run, "sensor_task", CONFIG_APP_SENSOR_TASK_STACK_SIZE,
      nullptr, CONFIG_APP_SENSOR_TASK_PRIORITY, &s_task_handle);

  if (result != pdPASS) {
    sensor_task_release_gpio();
    s_sample_queue = nullptr;
    vSemaphoreDelete(s_sensor_mutex);
    s_sensor_mutex = nullptr;
    return ESP_ERR_NO_MEM;
  }

  err = gpio_intr_enable(SENSOR_INT_GPIO);

  if (err != ESP_OK) {
    sensor_task_rollback_start();
    return err;
  }

  *task_handle = s_task_handle;

  ESP_LOGI(TAG, "Interrupt-driven sensor task started");
  return ESP_OK;
}

esp_err_t sensor_task_set_odr(mma8451q_odr_t odr) {
  if (s_task_handle == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = sensor_task_lock();

  if (err != ESP_OK) {
    return err;
  }

  err = mma8451q_set_odr(odr);

  if (err == ESP_OK) {
    s_current_odr = odr;
  }

  sensor_task_unlock();

  if (err != ESP_OK) {
    sensor_task_handle_read_error(err);
  }

  return err;
}

void sensor_task_stop(void) {
  if (s_task_handle == nullptr) {
    return;
  }

  (void)gpio_intr_disable(SENSOR_INT_GPIO);

  s_stop_waiter = xTaskGetCurrentTaskHandle();
  s_stop_requested = true;
  xTaskNotifyGive(s_task_handle);
  (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  sensor_task_release_gpio();

  const esp_err_t lock_err = sensor_task_lock();
  if (lock_err == ESP_OK) {
    const esp_err_t err = mma8451q_enable_data_ready_interrupt(false);
    sensor_task_unlock();

    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to disable DATA_READY: %s", esp_err_to_name(err));
    }
  }

  if (s_sensor_mutex != nullptr) {
    vSemaphoreDelete(s_sensor_mutex);
    s_sensor_mutex = nullptr;
  }

  s_task_handle = nullptr;
  s_sample_queue = nullptr;
  s_stop_waiter = nullptr;

  s_consecutive_failures = 0U;
  s_next_recovery_time = 0U;
  ESP_LOGI(TAG, "Sensor task stopped");
}
