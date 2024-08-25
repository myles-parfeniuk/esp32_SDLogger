#include "SDLogger.hpp"

SDLogger::SDLogger(sd_logger_config_t cfg)
    : initialized(false)
    , mounted(false)
    , cfg(cfg)
    , pdrv(FF_DRV_NOT_USED)
{
    spi_bus_config_t spi_bus_cfg = {.mosi_io_num = cfg.io_mosi,
            .miso_io_num = cfg.io_miso,
            .sclk_io_num = cfg.io_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4000};

    cfg.sdmmc_host.max_freq_khz = cfg.sclk_speed_hz / 100000UL;

    ESP_ERROR_CHECK(spi_bus_initialize(static_cast<spi_host_device_t>(cfg.sdmmc_host.slot), &spi_bus_cfg, SDSPI_DEFAULT_DMA));

    slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = cfg.io_cs;
    slot_cfg.host_id = static_cast<spi_host_device_t>(cfg.sdmmc_host.slot);
}

SDLogger::~SDLogger()
{
    close_all_files();
}

bool SDLogger::init()
{
    const char* SUB_TAG = "SD->init()";

    esp_err_t err = ESP_OK;
    int card_hdl = -1;

    err = (cfg.sdmmc_host.init)();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: Host init failed.", SUB_TAG);
        return initialized;
    }

    err = sdspi_host_init_device(&slot_cfg, &card_hdl);
    if (err != ESP_OK)
    {
        // de-initialize host
        if (cfg.sdmmc_host.flags & SDMMC_HOST_FLAG_DEINIT_ARG)
            (cfg.sdmmc_host.deinit_p)(cfg.sdmmc_host.slot);
        else
            (cfg.sdmmc_host.deinit)();

        ESP_LOGE(TAG, "%s: Slot init failed.", SUB_TAG);
        return initialized;
    }

    if (card_hdl != cfg.sdmmc_host.slot)
        cfg.sdmmc_host.slot = card_hdl;

    card.host.command_timeout_ms = 4000U;

    for (int trials = 0; trials < 3; trials++)
    {
        // sdmmc_card_init can take awhile to run, delay here to reset task watchdog and give time for init call
        vTaskDelay(10 / portTICK_PERIOD_MS);
        err = sdmmc_card_init(&cfg.sdmmc_host, &card);

        if (err == ESP_OK)
            break;
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: sdmmc_card_init() call failure.", SUB_TAG);
        return initialized;
    }

    initialized = true;
    return initialized;
}

bool SDLogger::mount(size_t unit_size, int max_open_files, const char* path)
{
    const char* SUB_TAG = "SD->mount()";

    esp_err_t err = ESP_OK;
    FRESULT res = FR_OK;

    if (!initialized)
    {
        ESP_LOGE(TAG, "%s: Card not initialized.", SUB_TAG);
        return false;
    }

    if (mounted)
    {
        ESP_LOGE(TAG, "%s: Drive already mounted.", SUB_TAG);
        return false;
    }

    if (open_files.size() > 0)
        close_all_files();

    this->max_open_files = max_open_files;

    if (strlen(path) + 1 > MAX_ROOT_PATH_SZ)
    {
        ESP_LOGE(TAG, "%s: Max root path length exceeded.", SUB_TAG);
        return false;
    }

    strcpy(root_path, path);

    pdrv = FF_DRV_NOT_USED;
    if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == FF_DRV_NOT_USED)
    {
        ESP_LOGE(TAG, "%s: Max volumes already mounted.", SUB_TAG);
        return false;
    }

    ff_diskio_register_sdmmc(pdrv, &card);
    drv[0] = static_cast<char>('0' + pdrv);

    err = esp_vfs_fat_register(root_path, drv, max_open_files, &fs);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: esp_vfs_fat_register() call failed 0x(%x)", SUB_TAG, err);
        unmount();
        return false;
    }

    res = f_mount(fs, drv, 1);
    if (res != FR_OK)
    {
        print_fatfs_error(res, SUB_TAG, "f_mount()");
        return false;
    }

    mounted = true;

    load_info(); // ignore return statement, info can fail to load but card can still be mounted and usable

    return true;
}

bool SDLogger::unmount()
{
    const char* SUB_TAG = "SD->unmount()";
    FRESULT res = FR_OK;

    if (!mounted)
    {
        ESP_LOGW(TAG, "%s: Drive already unmounted", SUB_TAG);
        return false;
    }

    if (open_files.size() > 0)
        close_all_files();

    res = f_mount(nullptr, drv, 0); // unregister file system object and unmount

    if (res != FR_OK)
    {
        print_fatfs_error(res, SUB_TAG, "f_mount()");
    }
    else
    {
        ff_diskio_unregister(pdrv);
        esp_vfs_fat_unregister_path(root_path);
        mounted = false;
    }

    return (res == FR_OK);
}

bool SDLogger::is_mounted()
{
    return mounted;
}

bool SDLogger::is_initialized()
{
    return initialized;
}

bool SDLogger::format(size_t unit_size)
{
    const constexpr char* SUB_TAG = "SD->format()";
    FRESULT res = FR_OK;
    const constexpr size_t work_buff_sz = 4096;
    void* work_buff = nullptr;
    size_t alloc_unit_sz = 0;

    if (!initialized)
    {
        ESP_LOGE(TAG, "%s: Card not initialized.", SUB_TAG);
        return false;
    }

    res = f_mount(nullptr, drv, 0);
    if (res != FR_OK)
    {
        ESP_LOGE(TAG, "%s: Unmount failed (%d)", SUB_TAG, res);
        return false;
    }

    work_buff = ff_memalloc(work_buff_sz);
    if (work_buff == nullptr)
    {
        ESP_LOGE(TAG, "%s: No heap memory available for work buffer.", SUB_TAG);
        return false;
    }

    alloc_unit_sz = esp_vfs_fat_get_allocation_unit_size(card.csd.sector_size, unit_size);

    if (!mounted)
    {
        pdrv = FF_DRV_NOT_USED;
        if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == FF_DRV_NOT_USED)
        {
            ESP_LOGE(TAG, "%s: Max volumes already mounted, could not take sdmmc driver resource.", SUB_TAG);
            return false;
        }

        ff_diskio_register_sdmmc(pdrv, &card);
        drv[0] = static_cast<char>('0' + pdrv);
    }

    // partition sd card
    LBA_t plist[] = {100, 0, 0, 0}; // format entire drive, see f_fdisk documentation on elm-chan.org

    res = f_fdisk(pdrv, plist, work_buff);
    if (res != FR_OK)
    {
        print_fatfs_error(res, SUB_TAG, "f_fdisk()");
        free(work_buff);

        if (!mounted)
            ff_diskio_unregister(pdrv);

        return false;
    }

    // make the file system
    const MKFS_PARM opt = {static_cast<BYTE>(FM_ANY), 0, 0, 0, alloc_unit_sz};
    res = f_mkfs(drv, &opt, work_buff, work_buff_sz);
    free(work_buff);

    if (res != FR_OK)
    {
        print_fatfs_error(res, SUB_TAG, "f_mkfs()");

        if (!mounted)
            ff_diskio_unregister(pdrv);

        return false;
    }

    if (!mounted)
        ff_diskio_unregister(pdrv);

    return true;
}

bool SDLogger::get_info(sd_info_t& sd_info)
{
    const constexpr char* SUB_TAG = "SD->get_info()";

    if (!usability_check(SUB_TAG))
        return false;

    if (!info.initialized)
    {
        ESP_LOGE(TAG, "%s: Card info never loaded.", SUB_TAG);
        return false;
    }

    sd_info = info;

    return true;
}

void SDLogger::print_info()
{
    const constexpr char* SUB_TAG = "SD->print_info()";

    if (!usability_check(SUB_TAG))
        return;

    if (!info.initialized)
    {
        ESP_LOGE(TAG, "%s: Card info never loaded.", SUB_TAG);
        return;
    }

    ESP_LOGI(TAG,
            "\n ------ SD Info ------ \n"
            "Name: %s \n"
            "Type: %s \n"
            "Speed (MHz): %.2f \n"
            "Size (MB): %lu \n"
            "SSR->Bus Width: %d \n"
            "CSD->Version: %d \n"
            "CSD->Sector Size: %d \n"
            "CSD->Capacity (bytes): %llu \n"
            "CSD->Read Block Length: %d \n"
            "-------------------- \n",
            info.name, info.type, info.speed_mhz, info.size_mb, info.ssr_bus_width, info.csd.ver, info.csd.sector_sz, info.csd.capacity,
            info.csd.read_bl_len);
}

const char* SDLogger::get_root_path()
{
    const constexpr char* SUB_TAG = "SD->get_root_path()";

    if (!usability_check(SUB_TAG))
        return nullptr;
    else
        return root_path;
}

void SDLogger::print_fatfs_error(FRESULT f_res, const char* SUBTAG, const char* fatfs_fxn)
{
    char res_str[40];
    fatfs_res_to_str(f_res, res_str);
    ESP_LOGE(TAG, "%s: %s did not return FR_OK, FRESULT: %s", SUBTAG, fatfs_fxn, res_str);
}

bool SDLogger::posix_perms_2_fatfs_perms(const char* posix_perms, uint8_t& fatfs_perms)
{
    auto idx = permission_flag_map.find(posix_perms);

    if (idx != permission_flag_map.end())
    {
        fatfs_perms = idx->second;
        return true;
    }

    return false;
}

bool SDLogger::path_exists(const char* path, const char* SUB_TAG, bool suppress_no_dir_warning)
{
    FRESULT res = FR_OK;

    if (!usability_check(SUB_TAG))
        return false;

    res = f_stat(path, nullptr);

    if (res != FR_OK && res != FR_NO_FILE)
        print_fatfs_error(res, SUB_TAG, "f_stat()");
    else if (res == FR_NO_FILE && !suppress_no_dir_warning)
        ESP_LOGW(TAG, "%s: File or path does not exist.", SUB_TAG);

    return (res == FR_OK);
}

void SDLogger::fatfs_res_to_str(FRESULT f_res, char* dest_str)
{
    switch (f_res)
    {
    case FR_OK:
        sprintf(dest_str, "FR_OK");
        break;
    case FR_DISK_ERR:
        sprintf(dest_str, "FR_DISK_ERR");
        break;
    case FR_INT_ERR:
        sprintf(dest_str, "FR_INT_ERR");
        break;
    case FR_NOT_READY:
        sprintf(dest_str, "FR_NOT_READY");
        break;
    case FR_NO_FILE:
        sprintf(dest_str, "FR_NO_FILE");
        break;
    case FR_NO_PATH:
        sprintf(dest_str, "FR_NO_PATH");
        break;
    case FR_INVALID_NAME:
        sprintf(dest_str, "FR_INVALID_NAME");
        break;
    case FR_DENIED:
        sprintf(dest_str, "FR_DENIED");
        break;
    case FR_EXIST:
        sprintf(dest_str, "FR_EXIST");
        break;
    case FR_INVALID_OBJECT:
        sprintf(dest_str, "FR_INVALID_OBJECT");
        break;
    case FR_WRITE_PROTECTED:
        sprintf(dest_str, "FR_WRITE_PROTECTED");
        break;
    case FR_INVALID_DRIVE:
        sprintf(dest_str, "FR_INVALID_DRIVE");
        break;
    case FR_NOT_ENABLED:
        sprintf(dest_str, "FR_NOT_ENABLED");
        break;
    case FR_NO_FILESYSTEM:
        sprintf(dest_str, "FR_NO_FILESYSTEM");
        break;
    case FR_MKFS_ABORTED:
        sprintf(dest_str, "FR_MKFS_ABORTED");
        break;
    case FR_TIMEOUT:
        sprintf(dest_str, "FR_TIMEOUT");
        break;
    case FR_LOCKED:
        sprintf(dest_str, "FR_LOCKED");
        break;
    case FR_NOT_ENOUGH_CORE:
        sprintf(dest_str, "FR_NOT_ENOUGH_CORE");
        break;
    case FR_TOO_MANY_OPEN_FILES:
        sprintf(dest_str, "FR_TOO_MANY_OPEN_FILES");
        break;
    case FR_INVALID_PARAMETER:
        sprintf(dest_str, "FR_INVALID_PARAMETER");
        break;
    default:
        sprintf(dest_str, "UNKNOWN_CODE");
        break;
    }
}

bool SDLogger::open_file(SDFile file, const char* permissions)
{
    const constexpr char* SUB_TAG = "SD->open_file()";
    char full_path[100];
    FRESULT res;
    uint8_t fatfs_mode = 0;

    if (!usability_check(SUB_TAG))
        return false;

    if (!file || !file->initialized)
    {
        ESP_LOGE(TAG, "%s: File not correctly initialized.", SUB_TAG);
        return false;
    }

    if (open_files.size() + 1 > max_open_files)
    {
        ESP_LOGE(TAG, "%s: Max files already opened.", SUB_TAG);
        return false;
    }

    if (!posix_perms_2_fatfs_perms(permissions, fatfs_mode))
    {
        ESP_LOGE(TAG, "%s: Invalid posix permission flag.", SUB_TAG);
        return false;
    }

    // build directory path if it does not exist
    if (strcmp(file->directory_path, "") != 0)
    {
        if (!path_exists(file->directory_path, SUB_TAG, true))
            if (!build_path(file->directory_path))
            {
                ESP_LOGE(TAG, "%s: Directory does not exist and path failed to build.", SUB_TAG);
            }
    }

    // append root path to file path to create the full path
    strcpy(full_path, root_path);
    strcat(full_path, file->path);

    // open the file
    res = f_open(&file->stream, file->path, fatfs_mode);
    if (res != FR_OK)
    {
        print_fatfs_error(res, SUB_TAG, "f_open()");
        return false;
    }

    // add pointer newly opened file to open_files vector
    open_files.push_back(file);
    file->open = true;

    return true;
}

bool SDLogger::close_file(SDFile file)
{
    const constexpr char* SUB_TAG = "SD->close_file()";
    bool found = false;
    int idx = 0;
    FRESULT res = FR_OK;

    if (!usability_check(SUB_TAG))
        return false;

    if (!file || !file->initialized)
    {
        ESP_LOGE(TAG, "%s: File not correctly initialized.", SUB_TAG);
        return false;
    }

    if (open_files.size() != 0)
        for (int i = 0; i < open_files.size(); i++)
        {

            if (strcmp(open_files[i]->path, file->path) == 0)
            {
                res = f_close(&file->stream);
                if (res != FR_OK)
                {
                    print_fatfs_error(res, SUB_TAG, "f_close()");
                }
                idx = i;
                found = true;
            }
        }

    if (found)
    {
        open_files.erase(open_files.begin() + idx);
        open_files.shrink_to_fit();
        file->open = false;
    }
    else
    {
        ESP_LOGW(TAG, "%s:  No matching open file found for path: %s", SUB_TAG, file->get_path());
    }

    return found;
}

bool SDLogger::close_all_files()
{
    const constexpr char* SUB_TAG = "SD->close_all_files()";
    FRESULT res = FR_OK;

    for (SDFile& f : open_files)
    {
        res = f_close(&f->stream);

        if (res != FR_OK)
        {
            print_fatfs_error(res, SUB_TAG, "f_close()");
            return false;
        }

        f->open = false;
    }

    open_files.clear();
    open_files.shrink_to_fit();

    return true;
}

bool SDLogger::create_directory(const char* path, bool suppress_dir_exists_warning)
{
    const constexpr char* SUB_TAG = "SD->create_directory()";
    FRESULT res = FR_OK;

    if (!usability_check(SUB_TAG))
        return false;

    res = f_mkdir(path);

    if (res != FR_OK)
    {

        switch (res)
        {
        case FR_NO_PATH:
            // attempt to build the path if it doesn't exists
            if (!build_path(path))
            {
                ESP_LOGE(TAG, "%s: Path does not exist and could not be built.", SUB_TAG);
                return false;
            }
            break;

        case FR_EXIST:
            if (!suppress_dir_exists_warning)
                ESP_LOGW(TAG, "%s: Directory already exists.", SUB_TAG);
            return true;
            break;

        default:
            print_fatfs_error(res, SUB_TAG, "f_mkdir()");
            return false;
            break;
        }
    }

    return true;
}

bool SDLogger::file_exists(SDFile file)
{
    const constexpr char* SUB_TAG = "SD->file_exists()";

    if (!file || !file->initialized)
    {
        ESP_LOGE(TAG, "%s: File not correctly initialized.", SUB_TAG);
        return false;
    }

    if (!path_exists(file->path, SUB_TAG))
        return false;

    return true;
}

bool SDLogger::path_exists(const char* path)
{
    const constexpr char* SUB_TAG = "SD->path_exists()";

    if (!path_exists(path, SUB_TAG))
        return false;

    return true;
}

bool SDLogger::build_path(const char* path)
{
    const constexpr char* SUB_TAG = "SD->build_path()";

    const char* start = path;
    const char* end;
    size_t length;
    char* part = nullptr;
    char path_str[100] = "";

    while ((end = strchr(start, '/')) != nullptr)
    {
        length = end - start;

        if (length > 0)
        {
            part = static_cast<char*>(malloc(length));

            if (part == nullptr)
            {
                ESP_LOGE(TAG, "%s: No heap memory available to parse path.", SUB_TAG);
                return false;
            }

            part[length] = '\0';
            strncpy(part, start, length);
            strcat(path_str, "/");
            strcat(path_str, part);
            free(part);

            if (!path_exists(path_str, SUB_TAG, true))
                if (!create_directory(path_str, true))
                    return false;
        }

        start = end + 1;
    }

    if (start != nullptr)
    {

        part = static_cast<char*>(malloc(strlen(start)));

        if (part == nullptr)
        {
            ESP_LOGE(TAG, "%s: No heap memory available to parse path.", SUB_TAG);
            return false;
        }

        strcpy(part, start);
        strcat(path_str, "/");
        strcat(path_str, part);
        free(part);

        if (!path_exists(path_str))
            if (!create_directory(path_str, true))
                return false;
    }

    return true;
}

bool SDLogger::write(SDFile file, const char* data)
{
    const constexpr char* SUB_TAG = "SD->write()";

    FRESULT res = FR_OK;
    UINT bytes_written;

    if (!usability_check(SUB_TAG))
        return false;

    if (!file || !file->initialized)
    {
        ESP_LOGE(TAG, "%s: File not correctly initialized.", SUB_TAG);
        return false;
    }

    if (!file->open)
    {
        ESP_LOGE(TAG, "%s: File not open.", SUB_TAG);
        return false;
    }

    // write here
    res = f_write(&file->stream, data, strlen(data), &bytes_written);
    if (res != FR_OK)
    {
        print_fatfs_error(res, SUB_TAG, "f_write()");
    }

    return true;
}

bool SDLogger::write_line(SDFile file, const char* line)
{
    const constexpr char* SUB_TAG = "SD->write_line()";

    size_t line_length;
    size_t temp_buffer_sz;
    char* temp_buffer;
    FRESULT res;
    UINT bytes_written;

    if (!usability_check(SUB_TAG))
        return false;

    if (!file || !file->initialized)
    {
        ESP_LOGE(TAG, "%s: File not correctly initialized.", SUB_TAG);
        return false;
    }

    if (!file->open)
    {
        ESP_LOGE(TAG, "%s: File not open.", SUB_TAG);
        return false;
    }

    line_length = strlen(line);
    temp_buffer_sz = line_length + 2;
    temp_buffer = new char[temp_buffer_sz];

    if (temp_buffer == nullptr)
    {
        ESP_LOGE(TAG, "%s: No heap memory available for line buffer.", SUB_TAG);
        return false;
    }

    strncpy(temp_buffer, line, line_length);

    temp_buffer[line_length] = '\n';
    temp_buffer[line_length + 1] = '\0';

    res = f_write(&file->stream, temp_buffer, strlen(temp_buffer), &bytes_written);
    delete[] temp_buffer;

    if (res != FR_OK)
    {
        print_fatfs_error(res, SUB_TAG, "f_write()");
        return false;
    }

    return true;
}

bool SDLogger::parse_info(const char* info_buffer)
{
    char temp_buff[50];

    if (!parse_info_field(info_buffer, "Name", info.name, sizeof(info.name)))
        return false;

    if (!parse_info_field(info_buffer, "Type", info.type, sizeof(info.type)))
        return false;

    if (!parse_info_field(info_buffer, "Speed", temp_buff, sizeof(temp_buff)))
        return false;

    info.speed_mhz = strtof(temp_buff, nullptr);

    if (!parse_info_field(info_buffer, "Size", temp_buff, sizeof(temp_buff)))
        return false;

    info.size_mb = strtoul(temp_buff, nullptr, 10);

    if (!parse_info_field(info_buffer, "bus_width", temp_buff, sizeof(temp_buff)))
        return false;

    info.ssr_bus_width = static_cast<uint8_t>(strtoul(temp_buff, nullptr, 10));

    if (!parse_info_field(info_buffer, "ver", temp_buff, sizeof(temp_buff)))
        return false;

    info.csd.ver = static_cast<uint8_t>(strtoul(temp_buff, nullptr, 10));

    if (!parse_info_field(info_buffer, "sector_size", temp_buff, sizeof(temp_buff)))
        return false;

    info.csd.sector_sz = static_cast<uint16_t>(strtoul(temp_buff, nullptr, 10));

    if (!parse_info_field(info_buffer, "capacity", temp_buff, sizeof(temp_buff)))
        return false;

    info.csd.capacity = strtoull(temp_buff, nullptr, 10);

    if (!parse_info_field(info_buffer, "read_bl_len", temp_buff, sizeof(temp_buff)))
        return false;
    info.csd.read_bl_len = static_cast<uint8_t>(strtoul(temp_buff, nullptr, 10));

    info.initialized = true;

    return true;
}

bool SDLogger::parse_info_field(const char* info_buffer, const char* key, char* output, size_t output_sz)
{
    const char* key_start = strstr(info_buffer, key);
    if (!key_start)
        return false;

    key_start += strlen(key) + 1; // skip key string and colon

    const char* key_end = strchr(key_start, '\n');
    if (key_end)
    {

        size_t length = key_end - key_start;

        if (length >= output_sz)
            length = output_sz - 1;

        strncpy(output, key_start, length);
        output[length] = '\0';
    }
    else
    {
        strncpy(output, key_start, output_sz - 1);
        output[output_sz - 1] = '\0';
    }

    return true;
}

bool SDLogger::load_info()
{
    const constexpr char* SUB_TAG = "SD->load_info()";

    size_t buffer_sz = 1024;
    char* buffer = static_cast<char*>(malloc(buffer_sz));
    FILE* memstream;

    if (buffer == nullptr)
    {
        ESP_LOGE(TAG, "%s: No heap memory available for info buffer.", SUB_TAG);
        return false;
    }

    memstream = fmemopen(buffer, buffer_sz, "w");
    if (memstream == nullptr)
    {
        ESP_LOGW(TAG, "%s: Failed to open memory stream.", SUB_TAG);
        free(buffer);
        return false;
    }

    sdmmc_card_print_info(memstream, &card);
    fclose(memstream);

    // parse info
    if (!parse_info(buffer))
    {
        ESP_LOGW(TAG, "%s: Failed to parse info buffer.", SUB_TAG);
        free(buffer);
        return false;
    }

    free(buffer);

    return true;
}

bool SDLogger::usability_check(const char* SUB_TAG)
{
    if (!initialized)
    {
        ESP_LOGE(TAG, "%s: Card not initialized.", SUB_TAG);
        return false;
    }

    if (!mounted)
    {
        ESP_LOGE(TAG, "%s: No card mounted.", SUB_TAG);
        return false;
    }

    return true;
}

SDFile SDLogger::File::create(const char* path)
{
    auto instance = SDFile(new File());

    if (instance->init(path))
        return instance;
    else
        return nullptr;
}

SDLogger::File::File()
    : initialized(false)
    , open(false)
    , path(nullptr)
    , directory_path(nullptr)
{
}

SDLogger::File::~File()
{
    if (path)
        delete[] path;

    if (directory_path)
        delete[] directory_path;
}

bool SDLogger::File::init(const char* path)
{
    const constexpr char* SUB_TAG = "SDFile->init()";

    // reset fields if re-initializing
    initialized = false;
    open = false;

    initialized = path_parse(path, SUB_TAG);

    return initialized;
}

bool SDLogger::File::path_forbidden_char_check(const char* path, const char* SUB_TAG)
{

    std::array<char, 8> forbidden_chars = {'\\', ':', '*', '?', '"', '<', '>', '|'};
    uint8_t period_count = 0;

    for (size_t i = 0; i < strlen(path); i++)
    {
        if (path[i] == '.')
            period_count++;

        if (period_count > 1)
        {
            ESP_LOGE(TAG, "%s: Invalid path, multiple '.' characters in path name.", SUB_TAG);
            return true;
        }

        for (const char forbidden_char : forbidden_chars)
            if (path[i] == forbidden_char)
            {
                ESP_LOGE(TAG, "%s: Invalid path, forbidden characters in path name.", SUB_TAG);
                return true;
            }
    }

    return false;
}

bool SDLogger::File::path_part_period_check(const char* part)
{
    for (size_t i = 0; i < strlen(part); i++)
    {
        if (part[i] == '.')
            return true;
    }

    return false;
}

bool SDLogger::File::create_path(const char* path, const char* SUB_TAG)
{
    size_t length = strlen(path);

    if (path_forbidden_char_check(path, SUB_TAG))
        return false;

    this->path = new char[length + 1];
    if (this->path == nullptr)
    {
        ESP_LOGE(TAG, "%s: No heap memory available for path.", SUB_TAG);
        return false;
    }

    // copy full path (w/o root directory) to file path member
    strcpy(this->path, "/");
    strcat(this->path + 1, path);

    return true;
}

bool SDLogger::File::create_directory_path(char* dir_path, const char* SUB_TAG)
{
    this->directory_path = new char[strlen(dir_path) + 1];
    if (this->directory_path == nullptr)
    {
        ESP_LOGE(TAG, "%s: No heap memory available for directory path.", SUB_TAG);
        return false;
    }

    this->directory_path[strlen(dir_path) + 1] = '\0';

    strcpy(this->directory_path, dir_path);

    return true;
}

bool SDLogger::File::path_tokenize_parts(const char* path, char* dir_path, char* file_name, const char* SUB_TAG)
{
    const char* start = nullptr;
    const char* end = nullptr;

    start = path;

    if (strlen(path) > 0)
        while ((end = strchr(start, '/')) != nullptr)
        {
            if (!path_tokenize_part(static_cast<size_t>(end - start), dir_path, start, SUB_TAG))
                return false;

            start = end + 1;
        }

    // check for final part
    if (strlen(start) > 0)
    {
        if (!path_tokenize_part(strlen(start) + 1, file_name, start, SUB_TAG))
            return false;

        if (!path_part_period_check(file_name))
        {
            ESP_LOGE(TAG, "%s: Invalid path, '.' character must be final part of file path to indicate file extension.", SUB_TAG);
            return false;
        }
    }

    return true;
}

bool SDLogger::File::path_tokenize_part(const size_t part_length, char* output_path, const char* start, const char* SUB_TAG)
{
    char* part;

    part = static_cast<char*>(malloc(part_length));

    if (part == nullptr)
    {
        ESP_LOGE(TAG, "%s: No heap memory available for parsing directory path.", SUB_TAG);
        return false;
    }

    part[part_length] = '\0';
    strncpy(part, start, part_length);
    strcat(output_path, "/");
    strcat(output_path, part);

    free(part);

    return true;
}

bool SDLogger::File::path_parse(const char* path, const char* SUB_TAG)
{
    char file_name[50] = "";
    char dir_path[100] = "";

    if (!create_path(path, SUB_TAG))
        return false;

    if (!path_tokenize_parts(path, dir_path, file_name, SUB_TAG))
        return false;

    // copy directory path (w/o root directory) to file directory member
    if (!create_directory_path(dir_path, SUB_TAG))
        return false;

    return true;
}

bool SDLogger::File::is_initialized()
{
    return initialized;
}

bool SDLogger::File::is_open()
{
    return open;
}

const char* SDLogger::File::get_path()
{
    return path;
}

const char* SDLogger::File::get_directory_path()
{
    return directory_path;
}
