#include "logger_task.h"
#include "app_state.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "sd_card.h"
#include "sdkconfig.h"
#include "status_led.h"

static const char *TAG = "logger_task";

typedef struct {
  bool enable;
} logger_control_command_t;

static QueueHandle_t s_sample_queue = nullptr;
static TaskHandle_t s_task_handle = nullptr;
static QueueHandle_t s_control_queue = nullptr;
static QueueHandle_t s_control_result_queue = nullptr;
static QueueSetHandle_t s_queue_set = nullptr;
static StaticSemaphore_t s_control_mutex_storage;
static SemaphoreHandle_t s_control_mutex = nullptr;

static constexpr TickType_t FLUSH_INTERVAL =
    pdMS_TO_TICKS(CONFIG_APP_LOGGER_FLUSH_INTERVAL_MS);
static constexpr TickType_t ACTIVITY_INTERVAL = pdMS_TO_TICKS(1000U);

static_assert(FLUSH_INTERVAL > 0,
              "Logger flush interval must produce at least one RTOS tick");
static_assert(ACTIVITY_INTERVAL > 0,
              "Activity interval must produce at least one RTOS tick");

static sd_card_record_t logger_task_make_record(const app_sample_t *sample) {
  return (sd_card_record_t){
      .timestamp_us = sample->timestamp_us,
      .x_raw = sample->acceleration.x_raw,
      .y_raw = sample->acceleration.y_raw,
      .z_raw = sample->acceleration.z_raw,
      .x_g = sample->acceleration.x_g,
      .y_g = sample->acceleration.y_g,
      .z_g = sample->acceleration.z_g,
  };
}

static void logger_task_disable_storage(const char *operation, esp_err_t err) {
  ESP_LOGE(TAG, "%s failed: %s", operation, esp_err_to_name(err));

  (void)app_state_record_sd_error();
  (void)app_state_set_storage(false, false);
  sd_card_deinit();
  (void)status_led_set_state(STATUS_LED_ERROR);
}

static esp_err_t logger_task_enable_logging(void) {
  app_state_snapshot_t state;
  esp_err_t err = app_state_get_snapshot(&state);

  if (err != ESP_OK) {
    return err;
  }

  if (!state.storage_ready) {
    return ESP_ERR_INVALID_STATE;
  }

  if (state.logging_enabled) {
    return ESP_OK;
  }

  err = sd_card_start();

  if (err != ESP_OK) {
    logger_task_disable_storage("SD logging start", err);
    return err;
  }

  (void)app_state_set_storage(true, true);
  (void)status_led_set_state(STATUS_LED_LOGGING);
  return ESP_OK;
}

static esp_err_t logger_task_disable_logging(void) {
  app_state_snapshot_t state;
  esp_err_t err = app_state_get_snapshot(&state);

  if (err != ESP_OK) {
    return err;
  }

  if (!state.storage_ready) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!state.logging_enabled) {
    return ESP_OK;
  }

  err = sd_card_stop();

  if (err != ESP_OK) {
    logger_task_disable_storage("SD logging stop", err);
    return err;
  }

  (void)app_state_set_storage(true, false);
  (void)status_led_set_state(STATUS_LED_READY);
  return ESP_OK;
}

static esp_err_t
logger_task_apply_control(const logger_control_command_t *command) {
  return command->enable ? logger_task_enable_logging()
                         : logger_task_disable_logging();
}

static bool logger_task_flush_if_needed(size_t *pending_records,
                                        TickType_t *last_flush_time) {
  const TickType_t now = xTaskGetTickCount();
  const bool count_reached =
      *pending_records >= CONFIG_APP_LOGGER_FLUSH_RECORD_COUNT;

  const bool interval_reached =
      *pending_records > 0 && (now - *last_flush_time) >= FLUSH_INTERVAL;

  if (!count_reached && !interval_reached) {
    return true;
  }

  esp_err_t err = sd_card_flush();

  if (err != ESP_OK) {
    logger_task_disable_storage("SD flush", err);
    return false;
  }

  *pending_records = 0;
  *last_flush_time = now;
  return true;
}

static void logger_task_process_sample(const app_sample_t *sample,
                                       size_t *pending_records,
                                       TickType_t *last_flush_time,
                                       TickType_t *last_activity_time) {
  app_state_snapshot_t state;
  esp_err_t err = app_state_get_snapshot(&state);

  if (err != ESP_OK || !state.logging_enabled) {
    return;
  }

  const sd_card_record_t record = logger_task_make_record(sample);
  err = sd_card_append(&record);

  if (err != ESP_OK) {
    logger_task_disable_storage("SD append", err);
    *pending_records = 0;
    return;
  }

  ++(*pending_records);
  (void)app_state_record_logged_sample();

  const TickType_t now = xTaskGetTickCount();

  if ((now - *last_activity_time) >= ACTIVITY_INTERVAL) {
    (void)status_led_activity_pulse();
    *last_activity_time = now;
  }

  if (!logger_task_flush_if_needed(pending_records, last_flush_time)) {
    *pending_records = 0;
  }
}

static void logger_task_process_control(size_t *pending_records,
                                        TickType_t *last_flush_time) {
  logger_control_command_t command;

  if (xQueueReceive(s_control_queue, &command, 0) != pdTRUE) {
    return;
  }

  const esp_err_t result = logger_task_apply_control(&command);

  if (result == ESP_OK) {
    *pending_records = 0;
    *last_flush_time = xTaskGetTickCount();
  }

  xQueueOverwrite(s_control_result_queue, &result);
}

static void logger_task_run(void *argument) {
  (void)argument;

  size_t pending_records = 0;
  TickType_t last_flush_time = xTaskGetTickCount();
  TickType_t last_activity_time = last_flush_time;

  while (true) {
    TickType_t wait = portMAX_DELAY;

    if (pending_records > 0U) {
      const TickType_t elapsed = xTaskGetTickCount() - last_flush_time;
      wait = elapsed >= FLUSH_INTERVAL ? 0 : FLUSH_INTERVAL - elapsed;
    }

    QueueSetMemberHandle_t ready = xQueueSelectFromSet(s_queue_set, wait);

    if (ready == nullptr) {
      if (!logger_task_flush_if_needed(&pending_records, &last_flush_time)) {
        pending_records = 0;
      }
      continue;
    }

    if (ready == s_control_queue) {
      logger_task_process_control(&pending_records, &last_flush_time);
      continue;
    }

    if (ready == s_sample_queue) {
      app_sample_t sample;

      if (xQueueReceive(s_sample_queue, &sample, 0) == pdTRUE) {
        logger_task_process_sample(&sample, &pending_records, &last_flush_time,
                                   &last_activity_time);
      }
    }
  }
}

static void logger_task_delete_control_queues(void) {
  if (s_queue_set != nullptr) {
    vQueueDelete(s_queue_set);
    s_queue_set = nullptr;
  }

  if (s_control_queue != nullptr) {
    vQueueDelete(s_control_queue);
    s_control_queue = nullptr;
  }

  if (s_control_result_queue != nullptr) {
    vQueueDelete(s_control_result_queue);
    s_control_result_queue = nullptr;
  }

  if (s_control_mutex != nullptr) {
    vSemaphoreDelete(s_control_mutex);
    s_control_mutex = nullptr;
  }
}

static esp_err_t logger_task_create_control_queues(void) {
  s_control_mutex = xSemaphoreCreateMutexStatic(&s_control_mutex_storage);
  s_control_queue = xQueueCreate(1, sizeof(logger_control_command_t));

  s_control_result_queue = xQueueCreate(1, sizeof(esp_err_t));

  if (s_control_mutex == nullptr || s_control_queue == nullptr ||
      s_control_result_queue == nullptr) {
    logger_task_delete_control_queues();
    return ESP_ERR_NO_MEM;
  }

  s_queue_set = xQueueCreateSet(CONFIG_APP_SAMPLE_QUEUE_LENGTH + 1U);

  if (s_queue_set == nullptr) {
    logger_task_delete_control_queues();
    return ESP_ERR_NO_MEM;
  }

  BaseType_t result = xQueueAddToSet(s_sample_queue, s_queue_set);

  if (result == pdPASS) {
    result = xQueueAddToSet(s_control_queue, s_queue_set);
  }

  if (result != pdPASS) {
    logger_task_delete_control_queues();
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t logger_task_start(QueueHandle_t sample_queue,
                            TaskHandle_t *task_handle) {
  if (sample_queue == nullptr || task_handle == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_task_handle != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  s_sample_queue = sample_queue;

  esp_err_t err = logger_task_create_control_queues();

  if (err != ESP_OK) {
    s_sample_queue = nullptr;
    return err;
  }

  const BaseType_t result = xTaskCreate(
      logger_task_run, "logger_task", CONFIG_APP_LOGGER_TASK_STACK_SIZE,
      nullptr, CONFIG_APP_LOGGER_TASK_PRIORITY, &s_task_handle);

  if (result != pdPASS) {
    logger_task_delete_control_queues();
    s_sample_queue = nullptr;
    return ESP_ERR_NO_MEM;
  }

  *task_handle = s_task_handle;
  ESP_LOGI(TAG, "Logger task started");
  return ESP_OK;
}

esp_err_t logger_task_set_logging(bool enable) {
  if (s_task_handle == nullptr || s_control_queue == nullptr ||
      s_control_result_queue == nullptr || s_control_mutex == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(s_control_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  const logger_control_command_t command = {
      .enable = enable,
  };

  xQueueReset(s_control_result_queue);

  if (xQueueSend(s_control_queue, &command, portMAX_DELAY) != pdTRUE) {
    xSemaphoreGive(s_control_mutex);
    return ESP_FAIL;
  }

  esp_err_t result;

  if (xQueueReceive(s_control_result_queue, &result, portMAX_DELAY) != pdTRUE) {
    xSemaphoreGive(s_control_mutex);
    return ESP_FAIL;
  }

  xSemaphoreGive(s_control_mutex);
  return result;
}

void logger_task_stop(void) {
  if (s_task_handle == nullptr) {
    return;
  }

  vTaskDelete(s_task_handle);
  s_task_handle = nullptr;
  logger_task_delete_control_queues();
  s_sample_queue = nullptr;

  ESP_LOGI(TAG, "Logger task stopped");
}
