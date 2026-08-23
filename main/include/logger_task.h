#ifndef LOGGER_TASK_H
#define LOGGER_TASK_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the queue-consuming SD logger task.
 *
 * @param sample_queue Queue containing app_sample_t values.
 * @param task_handle Destination for the created task handle.
 *
 * @return ESP_OK on success or an argument, state, or allocation error.
 */
esp_err_t logger_task_start(QueueHandle_t sample_queue,
                            TaskHandle_t *task_handle);

/**
 * @brief Stop the logger task if it is running.
 */
void logger_task_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_TASK_H */
