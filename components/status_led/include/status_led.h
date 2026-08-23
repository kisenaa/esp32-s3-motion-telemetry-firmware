#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Visual states displayed by the onboard RGB status LED.
 */
typedef enum {
  STATUS_LED_OFF,
  STATUS_LED_INITIALIZING,
  STATUS_LED_READY,
  STATUS_LED_LOGGING,
  STATUS_LED_ERROR,
} status_led_state_t;

/**
 * @brief Initialize the onboard RGB LED and external activity LED.
 *
 * The RGB LED is set to STATUS_LED_INITIALIZING after successful
 * initialization. GPIO assignments are obtained from project Kconfig.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if the component is already initialized.
 *      - ESP_ERR_INVALID_ARG if a configured GPIO is invalid.
 *      - ESP_ERR_NO_MEM if an internal driver resource cannot be allocated.
 *      - Another ESP-IDF driver error on initialization failure.
 */
esp_err_t status_led_init(void);

/**
 * @brief Set the visual state displayed by the onboard RGB LED.
 *
 * @param state State to display.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if the component is not initialized.
 *      - ESP_ERR_INVALID_ARG if state is invalid.
 *      - Another LED-strip driver error if the update fails.
 */
esp_err_t status_led_set_state(status_led_state_t state);

/**
 * @brief Start a short, non-blocking pulse on the external activity LED.
 *
 * Calling this function while a pulse is active restarts the pulse duration.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if the component is not initialized.
 *      - Another GPIO or timer error if the pulse cannot be started.
 */
esp_err_t status_led_activity_pulse(void);

#ifdef __cplusplus
}
#endif

#endif /* STATUS_LED_H */
