#ifndef MMA8451Q_H
#define MMA8451Q_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MMA8451Q full-scale measurement ranges.
 *
 * Values correspond to the FS field in XYZ_DATA_CFG.
 */
typedef enum {
  MMA8451Q_RANGE_2G = 0,
  MMA8451Q_RANGE_4G = 1,
  MMA8451Q_RANGE_8G = 2,
} mma8451q_range_t;

/**
 * @brief MMA8451Q output data rates.
 *
 * Values correspond to the DR field in CTRL_REG1.
 */
typedef enum {
  MMA8451Q_ODR_800_HZ = 0,
  MMA8451Q_ODR_400_HZ = 1,
  MMA8451Q_ODR_200_HZ = 2,
  MMA8451Q_ODR_100_HZ = 3,
  MMA8451Q_ODR_50_HZ = 4,
  MMA8451Q_ODR_12_5_HZ = 5,
  MMA8451Q_ODR_6_25_HZ = 6,
  MMA8451Q_ODR_1_56_HZ = 7,
} mma8451q_odr_t;

/**
 * @brief One decoded three-axis acceleration sample.
 */
typedef struct {
  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;

  float x_g;
  float y_g;
  float z_g;
} mma8451q_sample_t;

/**
 * @brief Interrupt-source flags returned by the MMA8451Q.
 */
typedef enum {
  MMA8451Q_INTERRUPT_DATA_READY = (1U << 0),
  MMA8451Q_INTERRUPT_FREEFALL_MOTION = (1U << 2),
  MMA8451Q_INTERRUPT_PULSE = (1U << 3),
  MMA8451Q_INTERRUPT_ORIENTATION = (1U << 4),
  MMA8451Q_INTERRUPT_TRANSIENT = (1U << 5),
  MMA8451Q_INTERRUPT_FIFO = (1U << 6),
  MMA8451Q_INTERRUPT_AUTO_SLEEP = (1U << 7),
} mma8451q_interrupt_t;

/**
 * @brief Portrait or landscape position decoded from LAPO.
 */
typedef enum {
  MMA8451Q_ORIENTATION_PORTRAIT_UP = 0,
  MMA8451Q_ORIENTATION_PORTRAIT_DOWN = 1,
  MMA8451Q_ORIENTATION_LANDSCAPE_RIGHT = 2,
  MMA8451Q_ORIENTATION_LANDSCAPE_LEFT = 3,
} mma8451q_orientation_t;

/**
 * @brief Decoded portrait/landscape status.
 */
typedef struct {
  bool changed;
  bool lockout;
  bool back_facing;
  mma8451q_orientation_t orientation;
} mma8451q_orientation_status_t;

/**
 * @brief Initialize the I2C bus and configure the MMA8451Q.
 *
 * The function creates the I2C bus, probes the configured address, verifies
 * WHO_AM_I, applies the configured range and output data rate, and places
 * the sensor in active mode.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if already initialized.
 *      - ESP_ERR_NOT_FOUND if the configured device is absent or WHO_AM_I
 *        does not identify an MMA8451Q.
 *      - Another I2C driver error on failure.
 */
esp_err_t mma8451q_init(void);

/**
 * @brief Verify that the initialized device still identifies as an MMA8451Q.
 *
 * @return
 *      - ESP_OK when WHO_AM_I is readable and correct.
 *      - ESP_ERR_INVALID_STATE when the driver is not initialized.
 *      - ESP_ERR_NOT_FOUND when WHO_AM_I contains an unexpected value.
 *      - Another I2C error when the device cannot be read.
 */
esp_err_t mma8451q_check_connection(void);

/**
 * @brief Remove the sensor device and release the owned I2C bus.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if not initialized.
 *      - Another I2C driver error on cleanup failure.
 */
esp_err_t mma8451q_deinit(void);

/**
 * @brief Read and decode one 14-bit acceleration sample.
 *
 * @param sample Destination for the decoded sample.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_ARG if sample is NULL.
 *      - ESP_ERR_INVALID_STATE if the driver is not initialized.
 *      - Another I2C driver error on failure.
 */
esp_err_t mma8451q_read_sample(mma8451q_sample_t *sample);

/**
 * @brief Change the full-scale measurement range.
 *
 * The driver temporarily enters standby if the sensor is active.
 *
 * @param range New measurement range.
 *
 * @return ESP_OK on success or an argument, state, or I2C error.
 */
esp_err_t mma8451q_set_range(mma8451q_range_t range);

/**
 * @brief Change the sensor output data rate.
 *
 * The driver temporarily enters standby if the sensor is active.
 *
 * @param odr New output data rate.
 *
 * @return ESP_OK on success or an argument, state, or I2C error.
 */
esp_err_t mma8451q_set_odr(mma8451q_odr_t odr);

/**
 * @brief Enable or disable data-ready interrupts routed to INT1.
 *
 * @param enable True to enable and route data-ready to INT1; false to
 *               disable it.
 *
 * @return ESP_OK on success or a state or I2C error.
 */
esp_err_t mma8451q_enable_data_ready_interrupt(bool enable);

/**
 * @brief Read the sensor interrupt-source register.
 *
 * @param sources Destination for the interrupt-source bit mask.
 *
 * @return ESP_OK on success or an argument, state, or I2C error.
 */
esp_err_t mma8451q_read_interrupt_source(uint8_t *sources);

/**
 * @brief Enable or disable portrait/landscape detection.
 *
 * @param enable True to enable orientation detection; false to disable it.
 *
 * @return ESP_OK on success or a state or I2C error.
 */
esp_err_t mma8451q_enable_orientation(bool enable);

/**
 * @brief Read and decode the current portrait/landscape status.
 *
 * Reading this status clears the sensor's NEWLP flag.
 *
 * @param status Destination for decoded orientation status.
 *
 * @return ESP_OK on success or an argument, state, or I2C error.
 */
esp_err_t mma8451q_read_orientation(mma8451q_orientation_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* MMA8451Q_H */
