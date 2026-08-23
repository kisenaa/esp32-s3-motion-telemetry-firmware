#include "logger_task.h"
#include "app_state.h"
#include "esp_log.h"
#include "sd_card.h"
#include "sdkconfig.h"
#include "status_led.h"

static const char *TAG = "logger_task";

static QueueHandle_t s_sample_queue = nullptr;
static TaskHandle_t s_task_handle = nullptr;

static constexpr TickType_t FLUSH_INTERVAL =
    pdMS_TO_TICKS(CONFIG_APP_LOGGER_FLUSH_INTERVAL_MS);

static_assert(FLUSH_INTERVAL > 0,
              "Logger flush interval must produce at least one RTOS tick");

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

static void logger_task_run(void *argument) {
  (void)argument;

  size_t pending_records = 0;
  TickType_t last_flush_time = xTaskGetTickCount();

  while (true) {
    app_sample_t sample;
    xQueueReceive(s_sample_queue, &sample, portMAX_DELAY);

    app_state_snapshot_t state;
    esp_err_t err = app_state_get_snapshot(&state);

    if (err != ESP_OK || !state.logging_enabled) {
      continue;
    }

    const sd_card_record_t record = logger_task_make_record(&sample);
    err = sd_card_append(&record);

    if (err != ESP_OK) {
      logger_task_disable_storage("SD append", err);
      pending_records = 0;
      continue;
    }

    pending_records++;
    (void)app_state_record_logged_sample();

    if (!logger_task_flush_if_needed(&pending_records, &last_flush_time)) {
      pending_records = 0;
    }
  }
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

  BaseType_t result = xTaskCreate(
      logger_task_run, "logger_task", CONFIG_APP_LOGGER_TASK_STACK_SIZE,
      nullptr, CONFIG_APP_LOGGER_TASK_PRIORITY, &s_task_handle);

  if (result != pdPASS) {
    s_sample_queue = nullptr;
    s_task_handle = nullptr;
    return ESP_ERR_NO_MEM;
  }

  *task_handle = s_task_handle;
  ESP_LOGI(TAG, "Logger task started");
  return ESP_OK;
}

void logger_task_stop(void) {
  if (s_task_handle == nullptr) {
    return;
  }

  vTaskDelete(s_task_handle);
  s_task_handle = nullptr;
  s_sample_queue = nullptr;
}
