#include "app_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static StaticSemaphore_t s_mutex_storage;
static SemaphoreHandle_t s_mutex = nullptr;
static app_state_snapshot_t s_state;

static esp_err_t app_state_lock(void) {
  if (s_mutex == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  return xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_FAIL;
}

static void app_state_unlock(void) { xSemaphoreGive(s_mutex); }

static esp_err_t app_state_increment(uint64_t *counter) {
  esp_err_t err = app_state_lock();

  if (err != ESP_OK) {
    return err;
  }

  ++(*counter);
  app_state_unlock();
  return ESP_OK;
}

esp_err_t app_state_init(void) {
  if (s_mutex != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  memset(&s_state, 0, sizeof(s_state));
  s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_storage);

  return s_mutex != nullptr ? ESP_OK : ESP_ERR_NO_MEM;
}

void app_state_deinit(void) {
  if (s_mutex == nullptr) {
    return;
  }

  vSemaphoreDelete(s_mutex);
  s_mutex = nullptr;
  memset(&s_state, 0, sizeof(s_state));
}

esp_err_t app_state_set_storage(bool storage_ready, bool logging_enabled) {
  esp_err_t err = app_state_lock();

  if (err != ESP_OK) {
    return err;
  }

  s_state.storage_ready = storage_ready;
  s_state.logging_enabled = storage_ready && logging_enabled;

  app_state_unlock();
  return ESP_OK;
}

esp_err_t app_state_set_sensor(bool sensor_ready, uint32_t rate_millihz) {
  if (sensor_ready && rate_millihz == 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = app_state_lock();

  if (err != ESP_OK) {
    return err;
  }

  s_state.sensor_ready = sensor_ready;
  s_state.sample_rate_millihz = sensor_ready ? rate_millihz : 0U;

  app_state_unlock();
  return ESP_OK;
}

esp_err_t app_state_record_sample(const app_sample_t *sample) {
  if (sample == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = app_state_lock();

  if (err != ESP_OK) {
    return err;
  }

  s_state.latest_sample = *sample;
  s_state.latest_sample_valid = true;
  ++s_state.samples_acquired;

  app_state_unlock();
  return ESP_OK;
}

esp_err_t app_state_record_sensor_interrupts(uint32_t count) {
  if (count == 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = app_state_lock();

  if (err != ESP_OK) {
    return err;
  }

  s_state.sensor_interrupts += count;

  app_state_unlock();
  return ESP_OK;
}

esp_err_t app_state_record_logged_sample(void) {
  return app_state_increment(&s_state.samples_logged);
}

esp_err_t app_state_record_queue_drop(void) {
  return app_state_increment(&s_state.queue_drops);
}

esp_err_t app_state_record_i2c_error(void) {
  return app_state_increment(&s_state.i2c_errors);
}

esp_err_t app_state_record_sd_error(void) {
  return app_state_increment(&s_state.sd_errors);
}

esp_err_t app_state_get_snapshot(app_state_snapshot_t *snapshot) {
  if (snapshot == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = app_state_lock();

  if (err != ESP_OK) {
    return err;
  }

  *snapshot = s_state;

  app_state_unlock();
  return ESP_OK;
}
