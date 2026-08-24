#ifndef CONSOLE_CLI_H
#define CONSOLE_CLI_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool sensor_ready;
  bool storage_ready;
  bool logging_enabled;
  bool latest_sample_valid;

  int64_t timestamp_us;
  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;

  float x_g;
  float y_g;
  float z_g;

  uint32_t sample_rate_millihz;
  uint32_t queued_samples;
  uint32_t queue_capacity;
  uint64_t samples_acquired;
  uint64_t sensor_interrupts;
  uint64_t samples_logged;
  uint64_t queue_drops;
  uint64_t i2c_errors;
  uint64_t sd_errors;
} console_cli_runtime_state_t;

typedef esp_err_t (*console_cli_get_state_fn)(
    void *context, console_cli_runtime_state_t *state);

typedef esp_err_t (*console_cli_set_logging_fn)(void *context, bool enable);

typedef esp_err_t (*console_cli_set_rate_fn)(void *context,
                                             uint32_t rate_millihz);

typedef struct {
  void *context;
  console_cli_get_state_fn get_state;
  console_cli_set_logging_fn set_logging;
  console_cli_set_rate_fn set_rate;
} console_cli_config_t;

/**
 * @brief Initialize and start the interactive application console.
 *
 * @param config Runtime callbacks used by the registered commands.
 *
 * @return ESP_OK on success or an argument, state, console, or UART error.
 */
esp_err_t console_cli_init(const console_cli_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_CLI_H */
