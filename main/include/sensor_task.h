#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mma8451q.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start interrupt-driven motion acquisition.
 *
 * Configures the MMA8451Q INT1 GPIO, registers the minimal GPIO ISR,
 * creates the acquisition task, and enables sensor DATA_READY interrupts.
 *
 * @param sample_queue Queue receiving app_sample_t values.
 * @param task_handle Destination for the created task handle.
 *
 * @return ESP_OK on success or an argument, state, GPIO, sensor, or
 *         allocation error.
 */
esp_err_t sensor_task_start(QueueHandle_t sample_queue,
                            TaskHandle_t *task_handle);

/**
 * @brief Change the MMA8451Q output data rate safely at runtime.
 *
 * Serializes the register update against interrupt-driven sample reads.
 *
 * @param odr New supported MMA8451Q output data rate.
 *
 * @return ESP_OK on success or a state, argument, or I2C error.
 */
esp_err_t sensor_task_set_odr(mma8451q_odr_t odr);

/**
 * @brief Disable DATA_READY delivery and stop the sensor task.
 */
void sensor_task_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_TASK_H */
