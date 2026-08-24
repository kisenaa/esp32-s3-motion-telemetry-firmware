#ifndef LOGGER_TASK_H
#define LOGGER_TASK_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
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
 * @brief Request a runtime logging-state change.
 *
 * The logger task performs the SD operation in its own context.
 *
 * @param enable True to start a new CSV session; false to stop logging.
 *
 * @return ESP_OK on success or a state or SD-card error. */
esp_err_t logger_task_set_logging(bool enable);

/**
 * @brief Stop the logger task if it is running.
 */
void logger_task_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_TASK_H */
