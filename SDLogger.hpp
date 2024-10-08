#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <array>
#include <memory>
#include <unordered_map>

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
                using SDFile = std::shared_ptr<File>;

                static SDFile create(const char* path);
                ~File();
                bool init(const char* path);
                bool is_initialized();
                bool is_open();
                const char* get_path();
                const char* get_directory_path();

            private:
                File();
                bool path_tokenize_parts(const char* path, char* dir_path, char* file_name, const char* SUB_TAG);
                bool path_tokenize_part(const size_t part_length, char* output_path, const char* start, const char* SUB_TAG);
                bool path_parse(const char* path, const char* SUB_TAG);
                bool path_forbidden_char_check(const char* path, const char* SUB_TAG);
                bool path_part_period_check(const char* part);
                bool create_path(const char* path, const char* SUB_TAG);
                bool create_directory_path(char* dir_path, const char* SUB_TAG);
                bool initialized;
                bool open;
                FIL stream;
                char* path;
                char* directory_path;
                static const constexpr char* TAG = "SDLogger::File";

                friend class SDLogger;
        };

        using SDFile = std::shared_ptr<File>;

        SDLogger(sd_logger_config_t cfg = sd_logger_config_t());
        ~SDLogger();
        bool init();
        bool mount(size_t unit_size = 16 * 1024, int max_open_files = 5, const char* path = "/sdcard");
        bool unmount();
        bool format(size_t unit_size = 16 * 1024);
        bool open_file(SDFile file, const char* permissions = "a+");
        bool close_file(SDFile file);
        bool close_all_files();
        bool write(SDFile file, const char* data);
        bool write_line(SDFile file, const char* line);
        bool create_directory(const char* path, bool suppress_dir_exists_warning = false);
        bool delete_file(SDFile file);
        bool delete_directory(const char *path);
        bool file_exists(SDFile file);
        bool path_exists(const char* path);
        bool get_info(sd_info_t& sd_info);
        void print_info();
        bool is_initialized();
        bool is_mounted();
        const char* get_root_path();

    private:
        const std::unordered_map<const char*, uint8_t> permission_flag_map = {{"r", FA_READ}, {"r+", FA_READ | FA_WRITE},
                {"w", FA_CREATE_ALWAYS | FA_WRITE}, {"w+", FA_CREATE_ALWAYS | FA_WRITE | FA_READ}, {"a", FA_OPEN_APPEND | FA_WRITE},
                {"a+", FA_OPEN_APPEND | FA_WRITE | FA_READ}, {"wx", FA_CREATE_NEW | FA_WRITE}, {"w+x", FA_CREATE_NEW | FA_WRITE | FA_READ}};

        static const constexpr size_t SD_SECTOR_SZ = 512U;
        static const constexpr size_t MAX_ROOT_PATH_SZ = 40;
        static const constexpr char* TAG = "SDLogger";

        bool load_info();
        bool parse_info(const char* info_buffer);
        bool parse_info_field(const char* info_buffer, const char* key, char* output, size_t output_sz);
        bool usability_check(const char* SUB_TAG);
        bool build_path(const char* path);
        void fatfs_res_to_str(FRESULT f_res, char* dest_str);
        void print_fatfs_error(FRESULT f_res, const char* SUBTAG, const char* fatfs_fxn);
        bool posix_perms_2_fatfs_perms(const char* posix_perms, uint8_t& fatfs_perms);
        bool path_exists(const char* path, const char* SUB_TAG, bool suppress_no_dir_warning = false);
        bool get_and_register_free_drive(const char *SUB_TAG); 
        bool initialized;
        bool mounted;
        sd_logger_config_t cfg;
        sdspi_device_config_t slot_cfg;

        // unpacked vfs_fat_sd_ctx_t into class
        esp_vfs_fat_sdmmc_mount_config_t mount_cfg;
        FATFS* fs = NULL;
        sdmmc_card_t card;
        char root_path[MAX_ROOT_PATH_SZ];
        BYTE pdrv;
        char drv[3] = {0, ':', 0};
        uint16_t max_open_files;
        std::vector<SDFile> open_files;

        sd_info_t info;
};

typedef std::shared_ptr<SDLogger::File> SDFile;