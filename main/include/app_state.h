#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "mma8451q.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One timestamped motion sample passed through the logger queue.
 */
typedef struct {
  int64_t timestamp_us;
  mma8451q_sample_t acceleration;
} app_sample_t;

/**
 * @brief Thread-safe snapshot of current application state.
 */
typedef struct {
  bool storage_ready;
  bool logging_enabled;
  bool latest_sample_valid;

  app_sample_t latest_sample;

  uint64_t samples_acquired;
  uint64_t sensor_interrupts;
  uint64_t samples_logged;
  uint64_t queue_drops;
  uint64_t i2c_errors;
  uint64_t sd_errors;
} app_state_snapshot_t;

/**
 * @brief Initialize the shared application state and its mutex.
 *
 * @return ESP_OK on success, or ESP_ERR_INVALID_STATE if already initialized.
 */
esp_err_t app_state_init(void);

/**
 * @brief Release the shared-state mutex.
 *
 * Call only after all tasks using the state have stopped.
 */
void app_state_deinit(void);

/**
 * @brief Update storage availability and logging state.
 *
 * @param storage_ready True when the SD logger is available.
 * @param logging_enabled True when incoming samples should be logged.
 *
 * @return ESP_OK on success or ESP_ERR_INVALID_STATE before initialization.
 */
esp_err_t app_state_set_storage(bool storage_ready, bool logging_enabled);

/**
 * @brief Record a newly acquired sample.
 *
 * Updates the latest cached sample and increments the acquisition count.
 *
 * @param sample Newly acquired timestamped sample.
 *
 * @return ESP_OK on success or an argument/state error.
 */
esp_err_t app_state_record_sample(const app_sample_t *sample);

/**
 * @brief Add delivered DATA_READY interrupts to the runtime statistics.
 *
 * @param count Number of task notifications delivered by the GPIO ISR.
 *
 * @return ESP_OK on success or an argument/state error.
 */
esp_err_t app_state_record_sensor_interrupts(uint32_t count);

esp_err_t app_state_record_logged_sample(void);
esp_err_t app_state_record_queue_drop(void);
esp_err_t app_state_record_i2c_error(void);
esp_err_t app_state_record_sd_error(void);

/**
 * @brief Copy a consistent snapshot of application state.
 *
 * @param snapshot Destination for the copied state.
 *
 * @return ESP_OK on success or an argument/state error.
 */
esp_err_t app_state_get_snapshot(app_state_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* APP_STATE_H */
