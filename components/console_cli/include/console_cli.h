#ifndef CONSOLE_CLI_H
#define CONSOLE_CLI_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the interactive console CLI.
 *
 * Initializes the configured UART REPL, registers all application command
 * names, and starts the background CLI task. Commands whose subsystems are
 * unavailable report that state without accessing hardware.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if the CLI is already initialized.
 *      - Another ESP-IDF console or UART driver error on failure.
 */
esp_err_t console_cli_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_CLI_H */
