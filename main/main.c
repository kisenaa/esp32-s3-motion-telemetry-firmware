#include "app_state.h"
#include "console_cli.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/queue.h"
#include "freertos/task.h"
#include "logger_task.h"
#include "mma8451q.h"
#include "sd_card.h"
#include "sdkconfig.h"
#include "sensor_task.h"
#include "status_led.h"
#include <string.h>

static const char *const TAG = "main";

static QueueHandle_t s_sample_queue = nullptr;
static TaskHandle_t s_sensor_task = nullptr;
static TaskHandle_t s_logger_task = nullptr;
static sd_card_tail_t s_cli_tail_source;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wenum-compare"
static_assert(CONSOLE_CLI_FILE_LIST_LIMIT == SD_CARD_LIST_LIMIT,
              "CLI and storage list capacities must match");

static_assert(CONSOLE_CLI_TAIL_LINE_COUNT == SD_CARD_TAIL_LINE_COUNT,
              "CLI and storage tail capacities must match");

static_assert(CONSOLE_CLI_CSV_LINE_CAPACITY == SD_CARD_CSV_LINE_CAPACITY,
              "CLI and storage line capacities must match");

static uint32_t app_default_rate_millihz(void) {
  static constexpr uint32_t RATES[] = {
      800000U, 400000U, 200000U, 100000U, 50000U, 12500U, 6250U, 1560U,
  };

  return RATES[CONFIG_APP_MMA8451Q_DEFAULT_ODR];
}
#pragma GCC diagnostic pop

static esp_err_t app_odr_from_millihz(uint32_t millihz, mma8451q_odr_t *odr) {
  if (odr == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  switch (millihz) {
  case 800000U:
    *odr = MMA8451Q_ODR_800_HZ;
    return ESP_OK;
  case 400000U:
    *odr = MMA8451Q_ODR_400_HZ;
    return ESP_OK;
  case 200000U:
    *odr = MMA8451Q_ODR_200_HZ;
    return ESP_OK;
  case 100000U:
    *odr = MMA8451Q_ODR_100_HZ;
    return ESP_OK;
  case 50000U:
    *odr = MMA8451Q_ODR_50_HZ;
    return ESP_OK;
  case 12500U:
    *odr = MMA8451Q_ODR_12_5_HZ;
    return ESP_OK;
  case 6250U:
    *odr = MMA8451Q_ODR_6_25_HZ;
    return ESP_OK;
  case 1560U:
    *odr = MMA8451Q_ODR_1_56_HZ;
    return ESP_OK;
  default:
    return ESP_ERR_INVALID_ARG;
  }
}

static void app_cli_copy_sample(const app_state_snapshot_t *source,
                                console_cli_runtime_state_t *destination) {
  destination->latest_sample_valid = source->latest_sample_valid;
  destination->timestamp_us = source->latest_sample.timestamp_us;

  destination->x_raw = source->latest_sample.acceleration.x_raw;
  destination->y_raw = source->latest_sample.acceleration.y_raw;
  destination->z_raw = source->latest_sample.acceleration.z_raw;

  destination->x_g = source->latest_sample.acceleration.x_g;
  destination->y_g = source->latest_sample.acceleration.y_g;
  destination->z_g = source->latest_sample.acceleration.z_g;
}

static void app_cli_copy_counters(const app_state_snapshot_t *source,
                                  console_cli_runtime_state_t *destination) {
  destination->samples_acquired = source->samples_acquired;
  destination->sensor_interrupts = source->sensor_interrupts;
  destination->samples_logged = source->samples_logged;
  destination->queue_drops = source->queue_drops;
  destination->i2c_errors = source->i2c_errors;
  destination->sensor_reinitializations = source->sensor_reinitializations;
  destination->sd_errors = source->sd_errors;
}

static esp_err_t app_cli_get_state(void *context,
                                   console_cli_runtime_state_t *destination) {
  (void)context;

  if (destination == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  app_state_snapshot_t source;
  const esp_err_t err = app_state_get_snapshot(&source);
  if (err != ESP_OK) {
    return err;
  }

  memset(destination, 0, sizeof(*destination));

  destination->sensor_ready = source.sensor_ready;
  destination->storage_ready = source.storage_ready;
  destination->logging_enabled = source.logging_enabled;
  destination->sample_rate_millihz = source.sample_rate_millihz;

  destination->queued_samples =
      (uint32_t)uxQueueMessagesWaiting(s_sample_queue);
  destination->queue_capacity = CONFIG_APP_SAMPLE_QUEUE_LENGTH;

  app_cli_copy_sample(&source, destination);
  app_cli_copy_counters(&source, destination);
  return ESP_OK;
}

static esp_err_t app_cli_set_logging(void *context, bool enable) {
  (void)context;
  return logger_task_set_logging(enable);
}

static esp_err_t app_cli_set_rate(void *context, uint32_t rate_millihz) {
  (void)context;

  mma8451q_odr_t odr;
  esp_err_t err = app_odr_from_millihz(rate_millihz, &odr);

  if (err != ESP_OK) {
    return err;
  }
  err = sensor_task_set_odr(odr);

  if (err != ESP_OK) {
    return err;
  }

  return app_state_set_sensor(true, rate_millihz);
}

static esp_err_t app_cli_list_files(void *context,
                                    console_cli_file_list_t *destination) {
  (void)context;

  if (destination == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  sd_card_file_list_t source;
  const esp_err_t err = logger_task_list_files(&source);

  if (err != ESP_OK) {
    return err;
  }

  memset(destination, 0, sizeof(*destination));
  destination->count = source.count;

  for (size_t index = 0U; index < source.count; ++index) {
    destination->files[index].index = source.files[index].index;
    destination->files[index].size_bytes = source.files[index].size_bytes;
  }

  return ESP_OK;
}

static esp_err_t app_cli_tail_file(void *context, uint16_t index,
                                   console_cli_tail_t *destination) {
  (void)context;

  if (destination == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  const esp_err_t err = logger_task_tail_file(index, &s_cli_tail_source);

  if (err != ESP_OK) {
    return err;
  }

  memset(destination, 0, sizeof(*destination));
  destination->count = s_cli_tail_source.count;

  for (size_t line = 0U; line < s_cli_tail_source.count; ++line) {
    memcpy(destination->lines[line], s_cli_tail_source.lines[line],
           sizeof(destination->lines[line]));
  }

  return ESP_OK;
}

static esp_err_t app_cli_delete_file(void *context, uint16_t index) {
  (void)context;
  return logger_task_delete_file(index);
}

static esp_err_t app_cli_delete_all(void *context, size_t *deleted_count) {
  (void)context;
  return logger_task_delete_all(deleted_count);
}

static bool app_initialize_storage(void) {
  esp_err_t err = sd_card_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "SD initialization failed; logging disabled: %s",
             esp_err_to_name(err));
    (void)app_state_set_storage(false, false);
    return false;
  }

#ifdef CONFIG_APP_DEFAULT_LOGGING_ENABLED
  err = sd_card_start();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "CSV start failed; logging disabled: %s",
             esp_err_to_name(err));
    (void)sd_card_deinit();
    (void)app_state_set_storage(false, false);
    return false;
  }

  (void)app_state_set_storage(true, true);
  ESP_LOGI(TAG, "SD logging ready");
#else
  (void)app_state_set_storage(true, false);
  ESP_LOGI(TAG, "SD card ready; logging disabled");
#endif
  return true;
}

static void app_cleanup_startup_failure(bool storage_ready) {
  sensor_task_stop();
  logger_task_stop();

  if (s_sample_queue != nullptr) {
    vQueueDelete(s_sample_queue);
    s_sample_queue = nullptr;
  }

  if (storage_ready) {
    (void)sd_card_stop();
  }

  (void)sd_card_deinit();
  (void)mma8451q_deinit();
  app_state_deinit();
}

static esp_err_t app_start_pipeline(void) {
  s_sample_queue =
      xQueueCreate(CONFIG_APP_SAMPLE_QUEUE_LENGTH, sizeof(app_sample_t));

  if (s_sample_queue == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t err = logger_task_start(s_sample_queue, &s_logger_task);

  if (err != ESP_OK) {
    vQueueDelete(s_sample_queue);
    s_sample_queue = nullptr;
    return err;
  }

  err = sensor_task_start(s_sample_queue, &s_sensor_task);

  if (err != ESP_OK) {
    logger_task_stop();
    vQueueDelete(s_sample_queue);
    s_sample_queue = nullptr;
    return err;
  }

  return ESP_OK;
}

void app_main(void) {
  esp_err_t err = status_led_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Status LED initialization failed: %s", esp_err_to_name(err));
    return;
  }

  (void)status_led_set_state(STATUS_LED_INITIALIZING);

  err = app_state_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Application state initialization failed: %s",
             esp_err_to_name(err));
    (void)status_led_set_state(STATUS_LED_ERROR);
    return;
  }

  err = mma8451q_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MMA8451Q initialization failed: %s", esp_err_to_name(err));
    app_state_deinit();
    (void)status_led_set_state(STATUS_LED_ERROR);
    return;
  }

  err = app_state_set_sensor(true, app_default_rate_millihz());

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to record sensor state: %s", esp_err_to_name(err));
    (void)mma8451q_deinit();
    app_state_deinit();
    (void)status_led_set_state(STATUS_LED_ERROR);
    return;
  }

  const bool storage_ready = app_initialize_storage();

  err = app_start_pipeline();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "FreeRTOS pipeline startup failed: %s", esp_err_to_name(err));
    app_cleanup_startup_failure(storage_ready);
    (void)status_led_set_state(STATUS_LED_ERROR);
    return;
  }

  const console_cli_config_t cli_config = {
      .context = nullptr,
      .get_state = app_cli_get_state,
      .set_logging = app_cli_set_logging,
      .set_rate = app_cli_set_rate,
      .list_files = app_cli_list_files,
      .tail_file = app_cli_tail_file,
      .delete_file = app_cli_delete_file,
      .delete_all = app_cli_delete_all,
  };
  err = console_cli_init(&cli_config);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Console CLI unavailable: %s", esp_err_to_name(err));
    (void)status_led_set_state(STATUS_LED_ERROR);
    return;
  }

  status_led_state_t state = STATUS_LED_ERROR;

  if (storage_ready) {
#ifdef CONFIG_APP_DEFAULT_LOGGING_ENABLED
    state = STATUS_LED_LOGGING;
#else
    state = STATUS_LED_READY;
#endif
  }

  err = status_led_set_state(state);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set runtime LED state: %s", esp_err_to_name(err));
  }

  ESP_LOGI(TAG, "Motion telemetry pipeline running");
}
