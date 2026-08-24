#include "sd_card.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"
#include <dirent.h>
#include <errno.h> // IWYU pragma: keep
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

/* Config Used*/
/*
  - CONFIG_APP_SD_SPI_HOST
  - CONFIG_APP_SD_SPI_FREQUENCY_KHZ
  - CONFIG_APP_SD_MOSI_GPIO
  - CONFIG_APP_SD_MISO_GPIO
  - CONFIG_APP_SD_SCLK_GPIO
  - CONFIG_APP_SD_CS_GPIO
  - CONFIG_APP_SD_MOUNT_POINT
  - CONFIG_APP_SD_CSV_FILENAME
  - CONFIG_APP_SD_MAX_OPEN_FILES
  - CONFIG_APP_SD_ALLOCATION_UNIT_SIZE
  - CONFIG_APP_SD_FORMAT_IF_MOUNT_FAILED
*/

static const char *const TAG = "sd_card";

static constexpr char CSV_HEADER[] =
    "timestamp_us,x_raw,y_raw,z_raw,x_g,y_g,z_g\n";

static constexpr size_t CSV_PATH_SIZE = 128U;
static constexpr int SPI_UNUSED_GPIO = -1;
static constexpr int SD_MAX_TRANSFER_SIZE = 4096;

static sdmmc_card_t *s_card = nullptr;
static FILE *s_csv_file = nullptr;

static spi_host_device_t s_spi_host = (spi_host_device_t)CONFIG_APP_SD_SPI_HOST;

static bool s_spi_bus_initialized = false;
static bool s_initialized = false;

static char s_csv_path[CSV_PATH_SIZE];
static sd_card_tail_t s_tail_ordered;

static bool sd_card_parse_file_name(const char *name, uint16_t *index) {
  if (name == nullptr || index == nullptr) {
    return false;
  }

  const char *const prefix = CONFIG_APP_SD_CSV_FILENAME;
  const size_t prefix_length = strlen(prefix);

  if (strncmp(name, prefix, prefix_length) != 0 || name[prefix_length] != '_') {
    return false;
  }

  const char *const suffix = name + prefix_length + 1U;

  uint16_t parsed_index = 0U;
  for (size_t position = 0U; position < 4U; ++position) {
    if (suffix[position] < '0' || suffix[position] > '9') {
      return false;
    }

    parsed_index =
        (uint16_t)(parsed_index * 10U + (uint16_t)(suffix[position] - '0'));
  }

  if (strcmp(suffix + 4U, ".csv") != 0 || parsed_index == 0U) {
    return false;
  }

  *index = parsed_index;
  return true;
}

static esp_err_t sd_card_build_indexed_path(uint16_t index, char *path,
                                            size_t path_size) {
  if (index == 0U || index > SD_CARD_FILE_INDEX_MAX || path == nullptr ||
      path_size == 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  const int written =
      snprintf(path, path_size, "%s/%s_%04" PRIu16 ".csv",
               CONFIG_APP_SD_MOUNT_POINT, CONFIG_APP_SD_CSV_FILENAME, index);

  if (written < 0 || (size_t)written >= path_size) {
    return ESP_ERR_INVALID_SIZE;
  }

  return ESP_OK;
}

static esp_err_t sd_card_build_next_csv_path(void) {
  DIR *const dir = opendir(CONFIG_APP_SD_MOUNT_POINT);

  if (dir == nullptr) {
    ESP_LOGE(TAG, "Failed to open %s", CONFIG_APP_SD_MOUNT_POINT);
    return ESP_FAIL;
  }

  uint16_t max_index = 0U;
  struct dirent *entry;

  while ((entry = readdir(dir)) != nullptr) {
    uint16_t index;

    if (sd_card_parse_file_name(entry->d_name, &index) && index > max_index) {
      max_index = index;
    }
  }

  const int close_result = closedir(dir);

  if (close_result != 0) {
    return ESP_FAIL;
  }

  if (max_index >= SD_CARD_FILE_INDEX_MAX) {
    ESP_LOGE(TAG, "Maximum telemetry file index reached");
    return ESP_ERR_INVALID_SIZE;
  }

  return sd_card_build_indexed_path((uint16_t)(max_index + 1U), s_csv_path,
                                    sizeof(s_csv_path));
}

static esp_err_t sd_card_check_management_state(void) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  return s_csv_file == nullptr ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static void sd_card_insert_file(sd_card_file_list_t *result,
                                sd_card_file_info_t file) {
  size_t position = 0U;

  while (position < result->count &&
         result->files[position].index > file.index) {
    ++position;
  }

  if (position >= SD_CARD_LIST_LIMIT) {
    return;
  }

  const size_t new_count =
      result->count < SD_CARD_LIST_LIMIT ? result->count + 1U : result->count;

  for (size_t index = new_count; index > position + 1U; --index) {
    result->files[index - 1U] = result->files[index - 2U];
  }

  result->files[position] = file;
  result->count = new_count;
}

static esp_err_t sd_card_mount(void) {
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = (int)s_spi_host;
  host.max_freq_khz = CONFIG_APP_SD_SPI_FREQUENCY_KHZ;

  const spi_bus_config_t bus_config = {
      .mosi_io_num = CONFIG_APP_SD_MOSI_GPIO,
      .miso_io_num = CONFIG_APP_SD_MISO_GPIO,
      .sclk_io_num = CONFIG_APP_SD_SCLK_GPIO,
      .quadwp_io_num = SPI_UNUSED_GPIO,
      .quadhd_io_num = SPI_UNUSED_GPIO,
      .max_transfer_sz = SD_MAX_TRANSFER_SIZE,
  };

  esp_err_t err = spi_bus_initialize(s_spi_host, &bus_config, SPI_DMA_CH_AUTO);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(err));
    return err;
  }

  s_spi_bus_initialized = true;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.host_id = s_spi_host;
  slot_config.gpio_cs = CONFIG_APP_SD_CS_GPIO;

  const esp_vfs_fat_mount_config_t mount_config = {
#ifdef CONFIG_APP_SD_FORMAT_IF_MOUNT_FAILED
      .format_if_mount_failed = true,
#else
      .format_if_mount_failed = false,
#endif
      .max_files = CONFIG_APP_SD_MAX_OPEN_FILES,
      .allocation_unit_size = CONFIG_APP_SD_ALLOCATION_UNIT_SIZE,
  };

  err = esp_vfs_fat_sdspi_mount(CONFIG_APP_SD_MOUNT_POINT, &host, &slot_config,
                                &mount_config, &s_card);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(err));

    const esp_err_t free_err = spi_bus_free(s_spi_host);

    if (free_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to release SPI bus after mount failure: %s",
               esp_err_to_name(free_err));
    }

    if (free_err == ESP_OK) {
      s_spi_bus_initialized = false;
    }
    return err;
  }

  sdmmc_card_print_info(stdout, s_card);

  return ESP_OK;
}

esp_err_t sd_card_init(void) {
  if (s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (s_card != nullptr || s_spi_bus_initialized) {
    const esp_err_t cleanup_err = sd_card_deinit();
    if (cleanup_err != ESP_OK) {
      return cleanup_err;
    }
  }

  const esp_err_t err = sd_card_mount();

  if (err != ESP_OK) {
    return err;
  }

  s_initialized = true;

  ESP_LOGI(TAG, "SD card mounted at %s", CONFIG_APP_SD_MOUNT_POINT);
  ESP_LOGI(TAG, "Card name: %s", s_card->cid.name);

  return ESP_OK;
}

esp_err_t sd_card_start(void) {
  if (!s_initialized || s_csv_file != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t err = sd_card_build_next_csv_path();

  if (err != ESP_OK) {
    return err;
  }

  s_csv_file = fopen(s_csv_path, "w");

  if (s_csv_file == nullptr) {
    ESP_LOGE(TAG, "Failed to open %s", s_csv_path);
    return ESP_FAIL;
  }

  if (fputs(CSV_HEADER, s_csv_file) == EOF || fflush(s_csv_file) != 0) {
    ESP_LOGE(TAG, "Failed to write the CSV header");
    const int close_result = fclose(s_csv_file);
    s_csv_file = nullptr;

    if (close_result == 0 && remove(s_csv_path) != 0) {
      ESP_LOGW(TAG, "Failed to remove incomplete CSV file %s", s_csv_path);
    }
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "CSV logging started: %s", s_csv_path);
  return ESP_OK;
}

esp_err_t sd_card_append(const sd_card_record_t *record) {
  if (record == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_csv_file == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  const int written = fprintf(
      s_csv_file,
      "%" PRId64 ",%" PRId16 ",%" PRId16 ",%" PRId16 ",%.6f,%.6f,%.6f\n",
      record->timestamp_us, record->x_raw, record->y_raw, record->z_raw,
      (double)record->x_g, (double)record->y_g, (double)record->z_g);

  if (written < 0) {
    ESP_LOGE(TAG, "Failed to append telemetry record");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t sd_card_flush(void) {
  if (s_csv_file == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  if (fflush(s_csv_file) != 0) {
    ESP_LOGE(TAG, "Failed to flush telemetry data");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t sd_card_stop(void) {
  if (s_csv_file == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = sd_card_flush();

  if (fclose(s_csv_file) != 0) {
    ESP_LOGE(TAG, "Failed to close telemetry CSV file");
    result = ESP_FAIL;
  }

  s_csv_file = nullptr;

  if (result == ESP_OK) {
    ESP_LOGI(TAG, "CSV logging stopped");
  }

  return result;
}

esp_err_t sd_card_list_files(sd_card_file_list_t *result) {
  if (result == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = sd_card_check_management_state();

  if (err != ESP_OK) {
    return err;
  }

  memset(result, 0, sizeof(*result));

  DIR *const dir = opendir(CONFIG_APP_SD_MOUNT_POINT);

  if (dir == nullptr) {
    return ESP_FAIL;
  }

  struct dirent *entry;

  while ((entry = readdir(dir)) != nullptr) {
    uint16_t index;

    if (!sd_card_parse_file_name(entry->d_name, &index)) {
      continue;
    }

    char path[CSV_PATH_SIZE];
    err = sd_card_build_indexed_path(index, path, sizeof(path));

    struct stat file_status;

    if (err != ESP_OK || stat(path, &file_status) != 0) {
      (void)closedir(dir);
      return err != ESP_OK ? err : ESP_FAIL;
    }

    const sd_card_file_info_t file = {
        .index = index,
        .size_bytes = (uint64_t)file_status.st_size,
    };

    sd_card_insert_file(result, file);
  }

  return closedir(dir) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t sd_card_tail_file(uint16_t index, sd_card_tail_t *result) {
  if (result == nullptr || index == 0U || index > SD_CARD_FILE_INDEX_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = sd_card_check_management_state();

  if (err != ESP_OK) {
    return err;
  }

  char path[CSV_PATH_SIZE];
  err = sd_card_build_indexed_path(index, path, sizeof(path));

  if (err != ESP_OK) {
    return err;
  }

  FILE *const file = fopen(path, "r");

  if (file == nullptr) {
    return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
  }

  memset(result, 0, sizeof(*result));
  size_t total_lines = 0U;
  char line[SD_CARD_CSV_LINE_CAPACITY];

  while (fgets(line, sizeof(line), file) != nullptr) {
    const size_t length = strlen(line);

    if (length == sizeof(line) - 1U && line[length - 1U] != '\n' &&
        !feof(file)) {
      (void)fclose(file);
      return ESP_ERR_INVALID_SIZE;
    }

    memcpy(result->lines[total_lines % SD_CARD_TAIL_LINE_COUNT], line,
           length + 1U);
    ++total_lines;
  }

  if (ferror(file) != 0 || fclose(file) != 0) {
    return ESP_FAIL;
  }

  const size_t count = total_lines < SD_CARD_TAIL_LINE_COUNT
                           ? total_lines
                           : SD_CARD_TAIL_LINE_COUNT;

  const size_t first = total_lines - count;
  s_tail_ordered.count = count;

  for (size_t offset = 0U; offset < count; ++offset) {
    const size_t source = (first + offset) % SD_CARD_TAIL_LINE_COUNT;

    memcpy(s_tail_ordered.lines[offset], result->lines[source],
           sizeof(s_tail_ordered.lines[offset]));
  }

  *result = s_tail_ordered;
  return ESP_OK;
}

esp_err_t sd_card_delete_file(uint16_t index) {
  if (index == 0U || index > SD_CARD_FILE_INDEX_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = sd_card_check_management_state();

  if (err != ESP_OK) {
    return err;
  }

  char path[CSV_PATH_SIZE];
  err = sd_card_build_indexed_path(index, path, sizeof(path));

  if (err != ESP_OK) {
    return err;
  }

  if (remove(path) == 0) {
    return ESP_OK;
  }

  return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
}

esp_err_t sd_card_delete_all(size_t *deleted_count) {
  if (deleted_count == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = sd_card_check_management_state();

  if (err != ESP_OK) {
    return err;
  }

  *deleted_count = 0U;
  DIR *const dir = opendir(CONFIG_APP_SD_MOUNT_POINT);

  if (dir == nullptr) {
    return ESP_FAIL;
  }

  struct dirent *entry;

  while ((entry = readdir(dir)) != nullptr) {
    uint16_t index;

    if (!sd_card_parse_file_name(entry->d_name, &index)) {
      continue;
    }

    char path[CSV_PATH_SIZE];
    err = sd_card_build_indexed_path(index, path, sizeof(path));

    if (err != ESP_OK || remove(path) != 0) {
      (void)closedir(dir);
      return err != ESP_OK ? err : ESP_FAIL;
    }

    ++(*deleted_count);
  }

  return closedir(dir) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t sd_card_deinit(void) {
  esp_err_t result = ESP_OK;

  if (s_csv_file != nullptr) {
    const esp_err_t stop_err = sd_card_stop();

    if (stop_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to stop logging during deinitialization: %s",
               esp_err_to_name(stop_err));
      result = stop_err;
    }
  }

  if (s_card != nullptr) {
    const esp_err_t unmount_err =
        esp_vfs_fat_sdcard_unmount(CONFIG_APP_SD_MOUNT_POINT, s_card);

    if (unmount_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to unmount SD card: %s",
               esp_err_to_name(unmount_err));
      if (result == ESP_OK) {
        result = unmount_err;
      }
    } else {
      ESP_LOGI(TAG, "SD card unmounted");
      s_card = nullptr;
    }
  }

  if (s_card == nullptr && s_spi_bus_initialized) {
    const esp_err_t free_err = spi_bus_free(s_spi_host);

    if (free_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to release SPI bus: %s", esp_err_to_name(free_err));
      if (result == ESP_OK) {
        result = free_err;
      }
    } else {
      s_spi_bus_initialized = false;
    }
  }

  if (s_csv_file == nullptr && s_card == nullptr && !s_spi_bus_initialized) {
    s_initialized = false;
  }

  return result;
}
