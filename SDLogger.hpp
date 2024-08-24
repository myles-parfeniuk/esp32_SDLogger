#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <vector>
#include <array>
#include <memory>

// esp-idf includes
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#include "vfs_fat_internal.h"

typedef struct sd_logger_config_t
{
        gpio_num_t io_cd;   // io 4
        gpio_num_t io_cs;   // io 13
        gpio_num_t io_mosi; // io 15
        gpio_num_t io_miso; // io 2
        gpio_num_t io_sclk; // io 14
        uint32_t sclk_speed_hz;
        sdmmc_host_t sdmmc_host;

        sd_logger_config_t()
            : io_cd(static_cast<gpio_num_t>(CONFIG_ESP32_SDLOGGER_GPIO_CD))
            , io_cs(static_cast<gpio_num_t>(CONFIG_ESP32_SDLOGGER_GPIO_CS))
            , io_mosi(static_cast<gpio_num_t>(CONFIG_ESP32_SDLOGGER_GPIO_MOSI))
            , io_miso(static_cast<gpio_num_t>(CONFIG_ESP32_SDLOGGER_GPIO_MISO))
            , io_sclk(static_cast<gpio_num_t>(CONFIG_ESP32_SDLOGGER_GPIO_SCLK))
            , sclk_speed_hz(static_cast<uint32_t>(CONFIG_ESP32_SDLOGGER_SCLK_SPEED_HZ))
            , sdmmc_host(SDSPI_HOST_DEFAULT())
        {
        }

} sd_logger_config_t;

typedef struct csd_info_t
{
        uint8_t ver;
        uint16_t sector_sz;
        uint64_t capacity;
        uint8_t read_bl_len;
} csd_info_t;

typedef struct sd_info_t
{
        bool initialized;
        char name[50];
        char type[50];
        float speed_mhz;
        uint32_t size_mb;
        uint8_t ssr_bus_width;
        csd_info_t csd;

        sd_info_t()
            : initialized(false)
            , name({0})
            , type({0})
            , speed_mhz(0)
            , size_mb(0)
            , ssr_bus_width(0)
            , csd({0, 0, 0, 0})
        {
        }
} sd_info_t;

class SDLogger
{
    public:
        class File
        {
            public:
                File();
                ~File();
                bool init(const char* path);
                bool is_initialized();
                bool is_open();
                const char* get_path();
                const char* get_directory_path();

            private:
                bool path_tokenize_parts(const char* path, char* dir_path, char* file_name);
                bool path_tokenize_part(const size_t part_length, char* output_path, const char* start);
                bool path_parse(const char* path);
                bool path_forbidden_char_check(const char* path);
                bool path_part_period_check(const char* part);
                bool create_path(const char* path);
                bool create_directory_path(char* dir_path);
                bool initialized;
                bool open;
                FILE* stream;
                char* path;
                char* directory_path;
                static const constexpr char* TAG = "SDLogger::File";

                friend class SDLogger;
        };

        SDLogger(sd_logger_config_t cfg = sd_logger_config_t());
        ~SDLogger();
        bool init();
        bool mount(size_t unit_size = 16 * 1024, int max_open_files = 5, const char* path = "/sdcard");
        bool unmount();
        bool format(size_t unit_size = 16 * 1024);
        bool open_file(File& file);
        bool close_file(File& file);
        void close_all_files();
        bool create_directory(const char* path, bool suppress_dir_exists_warning = false);
        bool path_exists(const char* path);
        bool write(File& file, const char* data);
        bool write_line(File& file, const char* line);
        bool get_info(sd_info_t& sd_info);
        void print_info();
        bool is_initialized();
        bool is_mounted();
        const char* get_root_path();

    private:
        bool load_sd_info();
        bool parse_info(const char* info_buffer);
        bool parse_info_field(const char* info_buffer, const char* key, char* output, size_t output_sz);
        bool usability_check(const char* SUB_TAG);
        bool build_path(const char* path);
        void fatfs_res_to_str(FRESULT f_res, char *dest_str);
        void print_fatfs_error(FRESULT f_res, const char *SUBTAG, const char *fatfs_fxn);
        bool initialized;
        bool mounted;
        sd_logger_config_t cfg;
        sdspi_device_config_t slot_cfg;

        // unpacked vfs_fat_sd_ctx_t into class
        esp_vfs_fat_sdmmc_mount_config_t mount_cfg;
        FATFS* fs = NULL;
        sdmmc_card_t card;
        char* root_path;
        BYTE pdrv;
        char drv[3] = {0, ':', 0};
        uint16_t max_open_files;
        std::vector<std::unique_ptr<File>> open_files;

        sd_info_t info;

        static const constexpr size_t SD_SECTOR_SZ = 512U;
        static const constexpr char* TAG = "SDLogger";
};