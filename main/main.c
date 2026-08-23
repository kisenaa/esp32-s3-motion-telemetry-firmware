/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "app_state.h"
#include "console_cli.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "logger_task.h"
#include "mma8451q.h"
#include "sd_card.h"
#include "sdkconfig.h"
#include "sensor_task.h"
#include "status_led.h"

static const char *TAG = "main";

static QueueHandle_t s_sample_queue = nullptr;
static TaskHandle_t s_sensor_task = nullptr;
static TaskHandle_t s_logger_task = nullptr;

static constexpr TickType_t STATE_TEST_DURATION = pdMS_TO_TICKS(2000);

static esp_err_t toogle_status_led(status_led_state_t state,
                                   const char *state_name) {
  ESP_LOGI(TAG, "Testing status LED to %s", state_name);

  esp_err_t err = status_led_set_state(state);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set status LED state: %s", esp_err_to_name(err));
    return err;
  }

  err = status_led_activity_pulse();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to pulse activity LED: %s", esp_err_to_name(err));
    return err;
  }

  vTaskDelay(STATE_TEST_DURATION);
  return ESP_OK;
}

void let_state_testing(void) {
  esp_err_t err = status_led_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Status LED initialization failed: %s", esp_err_to_name(err));
    return;
  }

  vTaskDelay(STATE_TEST_DURATION);

  while (true) {
    err = toogle_status_led(STATUS_LED_INITIALIZING, "initializing/yellow");

    if (err != ESP_OK) {
      return;
    }

    err = toogle_status_led(STATUS_LED_READY, "ready/green");

    if (err != ESP_OK) {
      return;
    }

    err = toogle_status_led(STATUS_LED_LOGGING, "logging/blue");

    if (err != ESP_OK) {
      return;
    }

    err = toogle_status_led(STATUS_LED_ERROR, "error/red");

    if (err != ESP_OK) {
      return;
    }

    err = toogle_status_led(STATUS_LED_OFF, "off");

    if (err != ESP_OK) {
      return;
    }
  }
}

static esp_err_t phase3_test_sd_logger(void) {
  esp_err_t err = sd_card_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "SD card initialization failed: %s", esp_err_to_name(err));
    return err;
  }

  err = sd_card_start();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "CSV logging start failed: %s", esp_err_to_name(err));
    sd_card_deinit();
    return err;
  }

  const sd_card_record_t test_records[] = {
      {
          .timestamp_us = esp_timer_get_time(),
          .x_raw = 12,
          .y_raw = -23,
          .z_raw = 4090,
          .x_g = 0.003f,
          .y_g = -0.006f,
          .z_g = 0.999f,
      },
      {
          .timestamp_us = esp_timer_get_time() + 10000,
          .x_raw = 18,
          .y_raw = -20,
          .z_raw = 4085,
          .x_g = 0.004f,
          .y_g = -0.005f,
          .z_g = 0.997f,
      },
      {
          .timestamp_us = esp_timer_get_time() + 20000,
          .x_raw = 21,
          .y_raw = -18,
          .z_raw = 4078,
          .x_g = 0.005f,
          .y_g = -0.004f,
          .z_g = 0.996f,
      },
  };

  for (size_t index = 0; index < sizeof(test_records) / sizeof(test_records[0]);
       ++index) {
    err = sd_card_append(&test_records[index]);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to append test record %u: %s", (unsigned)index,
               esp_err_to_name(err));
      (void)sd_card_stop();
      sd_card_deinit();
      return err;
    }
  }

  err = sd_card_stop();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "CSV logging stop failed: %s", esp_err_to_name(err));
    sd_card_deinit();
    return err;
  }

  sd_card_deinit();

  ESP_LOGI(TAG, "Phase 3 CSV test completed");
  return ESP_OK;
}

static const char *orientation_name(mma8451q_orientation_t orientation) {
  switch (orientation) {
  case MMA8451Q_ORIENTATION_PORTRAIT_UP:
    return "portrait up";

  case MMA8451Q_ORIENTATION_PORTRAIT_DOWN:
    return "portrait down";

  case MMA8451Q_ORIENTATION_LANDSCAPE_RIGHT:
    return "landscape right";

  case MMA8451Q_ORIENTATION_LANDSCAPE_LEFT:
    return "landscape left";

  default:
    return "unknown";
  }
}

static esp_err_t phase4_test_mma8451q(void) {
  esp_err_t err = mma8451q_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MMA8451Q initialization failed: %s", esp_err_to_name(err));
    return err;
  }

  err = mma8451q_enable_orientation(true);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable orientation detection: %s",
             esp_err_to_name(err));
    (void)mma8451q_deinit();
    return err;
  }

  err = mma8451q_enable_data_ready_interrupt(true);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable data-ready interrupt: %s",
             esp_err_to_name(err));
    (void)mma8451q_deinit();
    return err;
  }

  err = mma8451q_set_range(MMA8451Q_RANGE_8G);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set range to 8G: %s", esp_err_to_name(err));
    (void)mma8451q_deinit();
    return err;
  }

  for (size_t index = 0; index < 10; ++index) {
    mma8451q_sample_t sample;

    err = mma8451q_read_sample(&sample);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Sample read failed: %s", esp_err_to_name(err));
      (void)mma8451q_deinit();
      return err;
    }

    ESP_LOGI(TAG, "sample %u: raw=(%d, %d, %d), g=(%.3f, %.3f, %.3f)",
             (unsigned)index, sample.x_raw, sample.y_raw, sample.z_raw,
             (double)sample.x_g, (double)sample.y_g, (double)sample.z_g);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
  mma8451q_orientation_status_t orientation;

  err = mma8451q_read_orientation(&orientation);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Orientation read failed: %s", esp_err_to_name(err));
    (void)mma8451q_deinit();
    return err;
  }

  ESP_LOGI(TAG, "Orientation: %s, face: %s, changed: %s, lockout: %s",
           orientation_name(orientation.orientation),
           orientation.back_facing ? "back" : "front",
           orientation.changed ? "yes" : "no",
           orientation.lockout ? "yes" : "no");

  err = mma8451q_deinit();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MMA8451Q deinitialization failed: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "Phase 4 MMA8451Q test completed");
  return ESP_OK;
}

static bool app_initialize_storage(void) {
  esp_err_t err = sd_card_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "SD initialization failed; logging disabled: %s",
             esp_err_to_name(err));
    (void)app_state_set_storage(false, false);
    return false;
  }

  err = sd_card_start();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "CSV start failed; logging disabled: %s",
             esp_err_to_name(err));
    sd_card_deinit();
    (void)app_state_set_storage(false, false);
    return false;
  }

  (void)app_state_set_storage(true, true);
  ESP_LOGI(TAG, "SD logging ready");
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

  sd_card_deinit();
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
  // ESP_LOGI(TAG, "Starting status LED test application");
  // let_state_testing();

  // esp_err_t err = status_led_init();

  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Status LED initialization failed: %s",
  //   esp_err_to_name(err)); return;
  // }

  // err = phase3_test_sd_logger();

  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Phase 3 SD logger test failed: %s", esp_err_to_name(err));
  //   (void)status_led_set_state(STATUS_LED_ERROR);
  //   return;
  // }

  // err = phase4_test_mma8451q();

  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Phase 4 MMA8451Q test failed: %s", esp_err_to_name(err));
  //   (void)status_led_set_state(STATUS_LED_ERROR);
  //   return;
  // }

  // err = console_cli_init();

  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Console CLI initialization failed: %s",
  //            esp_err_to_name(err));
  //   (void)status_led_set_state(STATUS_LED_ERROR);
  //   return;
  // }

  // err = status_led_set_state(STATUS_LED_READY);

  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to set ready LED state: %s", esp_err_to_name(err));
  //   return;
  // }

  // ESP_LOGI(TAG, "Console CLI ready; type 'help' for commands");
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

  const bool storage_ready = app_initialize_storage();

  err = app_start_pipeline();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "FreeRTOS pipeline startup failed: %s", esp_err_to_name(err));
    app_cleanup_startup_failure(storage_ready);
    (void)status_led_set_state(STATUS_LED_ERROR);
    return;
  }

  err = console_cli_init();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Console CLI unavailable: %s", esp_err_to_name(err));
    (void)status_led_set_state(STATUS_LED_ERROR);
    return;
  }

  status_led_state_t state =
      storage_ready ? STATUS_LED_LOGGING : STATUS_LED_READY;

  err = status_led_set_state(state);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set runtime LED state: %s", esp_err_to_name(err));
  }

  ESP_LOGI(TAG, "Phase 5 acquisition pipeline running");
}
