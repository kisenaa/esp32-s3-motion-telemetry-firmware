#ifndef SD_CARD_H
#define SD_CARD_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One timestamped telemetry record written to the CSV file.
 */
typedef struct {
  int64_t timestamp_us;

  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;

  float x_g;
  float y_g;
  float z_g;
} sd_card_record_t;

/**
 * @brief Initialize the SDSPI bus and mount the SD card filesystem.
 *
 * GPIO assignments, SPI frequency, mount path, and FATFS settings are
 * obtained from project Kconfig.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if the component is already initialized.
 *      - An SPI, SD protocol, VFS, or FATFS error on failure.
 */
esp_err_t sd_card_init(void);

/**
 * @brief Start a new indexed CSV logging session.
 *
 * Scans the mount directory for existing indexed telemetry files, creates
 * the next available file, and writes its CSV header.
 *
 * Each successful stop/start cycle creates a new file.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if the card is not initialized or a logging
 *        session is already active.
 *      - ESP_ERR_INVALID_SIZE if no further index is available or the path
 *        exceeds the internal path capacity.
 *      - ESP_FAIL if the directory or CSV file cannot be opened.
 */
esp_err_t sd_card_start(void);

/**
 * @brief Append one telemetry record to the open CSV file.
 *
 * This function writes to the standard I/O buffer. Call sd_card_flush()
 * periodically rather than flushing after every record.
 *
 * @param record Telemetry record to append.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_ARG if record is NULL.
 *      - ESP_ERR_INVALID_STATE if logging has not started.
 *      - ESP_FAIL if writing fails.
 */
esp_err_t sd_card_append(const sd_card_record_t *record);

/**
 * @brief Flush buffered telemetry data to the filesystem.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if logging has not started.
 *      - ESP_FAIL if flushing fails.
 */
esp_err_t sd_card_flush(void);

/**
 * @brief Flush and close the telemetry CSV file.
 *
 * The SD card remains mounted, allowing logging to be started again later.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_STATE if logging has not started.
 *      - ESP_FAIL if flushing or closing the file fails.
 */
esp_err_t sd_card_stop(void);

/**
 * @brief Stop logging, unmount the filesystem, and release the SPI bus.
 *
 * Errors encountered during cleanup are logged internally. Calling this
 * function when the component is not initialized has no effect.
 */
void sd_card_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
