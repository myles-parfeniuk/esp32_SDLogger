#include "SDLogger.hpp"

SDLogger::SDLogger(sd_logger_config_t cfg)
    : initialized(false)
    , mounted(false)
    , cfg(cfg)
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
    esp_err_t err = ESP_OK;
    int card_hdl = -1;

    pdrv = FF_DRV_NOT_USED;

    if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == FF_DRV_NOT_USED)
    {
        ESP_LOGE(TAG, "Init Fail: Max volumes already mounted.");
        return initialized;
    }

    err = (cfg.sdmmc_host.init)();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Init Fail: Host init failed.");
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

        ESP_LOGE(TAG, "Init Failure: Slot init failed.");
        return initialized;
    }

    // sdmmc_card_init can take awhile to run, delay here to reset task watchdog and give time for init call
    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (card_hdl != cfg.sdmmc_host.slot)
        cfg.sdmmc_host.slot = card_hdl;

    card.host.command_timeout_ms = 1000U;
    err = sdmmc_card_init(&cfg.sdmmc_host, &card);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Init Failure: sdmmc_card_init() call failure.");
        return initialized;
    }

    ff_diskio_register_sdmmc(pdrv, &card);
    ESP_LOGI(TAG, "using pdrv=%i", pdrv);
    drv[0] = static_cast<char>('0' + pdrv);

    initialized = true;
    return initialized;
}

bool SDLogger::mount(size_t unit_size, int max_open_files, const char* path)
{
    esp_err_t err = ESP_OK;
    FRESULT res = FR_OK;
    pdrv = FF_DRV_NOT_USED;

    if (open_files.size() > 0)
        close_all_files();

    this->max_open_files = max_open_files;

    if (!initialized)
    {
        ESP_LOGE(TAG, "Mount Failure: Card not initialized.");
        return false;
    }

    root_path = static_cast<char*>(malloc(strlen(path) + 1));

    if (root_path == nullptr)
    {
        ESP_LOGE(TAG, "Mount Failure: No heap memory available for path.");
        return false;
    }

    strcpy(root_path, path);

    if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == FF_DRV_NOT_USED)
    {
        ESP_LOGE(TAG, "Init Fail: Max volumes already mounted.");
        return false;
    }

    err = esp_vfs_fat_register(root_path, drv, max_open_files, &fs);
    if (err != ESP_OK)
    {
        unmount();
        ESP_LOGE(TAG, "Mount Failure: esp_vfs_fat_register() call failed 0x(%x)", err);
        return false;
    }

    res = f_mount(fs, drv, 1);
    if (res != FR_OK)
    {
        ESP_LOGE(TAG, "Mount Failure: f_mount() call failed to mount card (%d)", res);
        return false;
    }

    mounted = true;

    load_sd_info(); // ignore return statement, info can fail to load but card can still be mounted and usable

    return true;
}

bool SDLogger::unmount()
{
    FRESULT res = FR_OK;

    if (open_files.size() > 0)
        close_all_files();

    if (fs)
        res = f_mount(nullptr, drv, 0); // unregister file system object and unmount

    esp_vfs_fat_unregister_path(root_path);

    mounted = false;
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
    FRESULT res = FR_OK;
    esp_err_t err = ESP_OK;
    const constexpr size_t work_buff_sz = 4096;
    void* work_buff = nullptr;
    size_t alloc_unit_sz = 0;

    pdrv = FF_DRV_NOT_USED;

    if (!initialized)
    {
        ESP_LOGE(TAG, "Format Failure: Card not initialized.");
        return false;
    }

    pdrv = ff_diskio_get_pdrv_card(&card);
    if (pdrv == FF_DRV_NOT_USED)
    {
        ESP_LOGE(TAG, "Format Failure: Card driver not registered.");
        return false;
    }

    // unmount
    res = f_mount(0, drv, 0);
    if (res != FR_OK)
    {
        ESP_LOGE(TAG, "Format Failure: Unmount failed (%d)", res);
        return false;
    }

    work_buff = ff_memalloc(work_buff_sz);
    if (work_buff == nullptr)
    {
        ESP_LOGE(TAG, "Format Failure: No heap memory available for work buffer.");
        return false;
    }

    alloc_unit_sz = esp_vfs_fat_get_allocation_unit_size(card.csd.sector_size, unit_size);

    LBA_t plist[] = {100, 0, 0, 0}; // format entire drive, see f_fdisk documentation on elm-chan.org
    res = f_fdisk(pdrv, plist, work_buff);
    if (res != FR_OK)
    {
        ESP_LOGE(TAG, "Format Failed: f_fdisk() call failed, FRESULT: (%d)", res);
        free(work_buff);
        return false;
    }

    const MKFS_PARM opt = {static_cast<BYTE>(FM_ANY), 0, 0, 0, alloc_unit_sz};
    res = f_mkfs(drv, &opt, work_buff, work_buff_sz);
    free(work_buff);

    if (res != FR_OK)
    {
        ESP_LOGE(TAG, "Format Failure: f_mkfs() call failed (%d)", res);
        return false;
    }

    return true;
}

bool SDLogger::get_info(sd_info_t& sd_info)
{
    if (!initialized)
    {
        ESP_LOGE(TAG, "Get Info Failure: Card not initialized.");
        return false;
    }

    if (!mounted)
    {
        ESP_LOGE(TAG, "Get Info Failure: No card mounted.");
        return false;
    }

    if (!info.initialized)
    {
        ESP_LOGE(TAG, "Get Info Failure: Card info never loaded.");
        return false;
    }

    sd_info = info;

    return true;
}

void SDLogger::print_info()
{
    if (!usability_check("Print Info Failure"))
        return;

    if (!info.initialized)
    {
        ESP_LOGE(TAG, "Print Info Failure: Card info never loaded.");
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
    if (!initialized)
        return nullptr;
    else
        return root_path;
}

bool SDLogger::open_file(File& file)
{
    char full_path[100];

    if (!usability_check("Open File Failure"))
        return false;

    if (!file.initialized)
    {
        ESP_LOGE(TAG, "Open File Failure: File not correctly initialized.");
        return false;
    }

    if (open_files.size() + 1 > max_open_files)
    {
        ESP_LOGE(TAG, "Open File Failure: Max files already opened.");
        return false;
    }

    if (strcmp(file.directory_path, "") != 0)
    {
        if (!directory_exists(file.directory_path))
            if (!build_path(file.directory_path))
            {
                ESP_LOGE(TAG, "Open File Failure: Directory does not exist and path failed to build.");
            }
    }

    strcpy(full_path, root_path);
    strcat(full_path, file.path);

    file.stream = fopen(full_path, "w");
    if (file.stream == nullptr)
    {
        ESP_LOGE(TAG, "Open File Failure: f_open() returned nullptr.");
        return false;
    }

    open_files.push_back(std::make_unique<File>(file));
    file.open = true;

    return true;
}

bool SDLogger::close_file(File& file)
{
    bool found = false;
    int idx = 0;

    if (!usability_check("Close File Failure"))
        return false;

    if (!file.initialized)
    {
        ESP_LOGE(TAG, "Close File Failure: File not correctly initialized.");
        return false;
    }

    if (open_files.size() != 0)
        for (int i = 0; i < open_files.size(); i++)
        {

            if (strcmp(open_files[i]->path, file.path) == 0)
            {
                fclose(file.stream);
                idx = i;
                found = true;
            }
        }

    if (found)
    {
        open_files.erase(open_files.begin() + idx);
        open_files.shrink_to_fit();
        file.open = false;
        file.stream = nullptr;
    }
    else
    {
        ESP_LOGW(TAG, "Close File Failure:  No matching file found for path: %s", file.get_path());
    }

    return found;
}

void SDLogger::close_all_files()
{

    for (std::unique_ptr<File>& f : open_files)
    {
        fclose(f->stream);
        f->open = false;
        f->stream = nullptr;
    }

    open_files.clear();
    open_files.shrink_to_fit();
}

bool SDLogger::create_directory(const char* path, bool suppress_dir_exists_warning)
{
    FRESULT res = FR_OK;

    if (!usability_check("Create Directory Failure"))
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
                ESP_LOGE(TAG, "Create Directory Failure: Path does not exist and could not be built.");
                return false;
            }
            break;

        case FR_INVALID_NAME:
            ESP_LOGE(TAG, "Create Directory Failure: Invalid name, path length exceeded (FF_MAX_LFN) or encoding error.  FRESULT: 0x%X", res);
            return false;
            break;

        case FR_DENIED:
            ESP_LOGE(TAG, "Create Directory Failure: Denied write, file most likely read only or opened without FA_WRITE.  FRESULT: 0x%X", res);
            break;

        case FR_EXIST:
            if (!suppress_dir_exists_warning)
                ESP_LOGW(TAG, "Create Directory Failure: Directory already exists. FRESULT: 0x%X", res);

        default:

            break;
        }
    }

    return true;
}

bool SDLogger::directory_exists(const char* path)
{
    if (!usability_check("Directory Existence Check Failure"))
        return false;

    FRESULT res = FR_OK;

    res = f_stat(path, nullptr);

    return (res == FR_OK);
}

bool SDLogger::build_path(const char* path)
{
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
                ESP_LOGE(TAG, "Build Path Failure: No heap memory available to parse path.");
                return false;
            }

            part[length] = '\0';
            strncpy(part, start, length);
            strcat(path_str, "/");
            strcat(path_str, part);
            free(part);

            if (!directory_exists(path_str))
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
            ESP_LOGE(TAG, "Build Path Failure: No heap memory available to parse path.");
            return false;
        }

        strcpy(part, start);
        strcat(path_str, "/");
        strcat(path_str, part);
        free(part);

        if (!directory_exists(path_str))
            if (!create_directory(path_str, true))
                return false;
    }

    return true;
}

bool SDLogger::write(File& file, const char* data)
{
    if (!usability_check("Write Failure"))
        return false;

    if (!file.initialized)
    {
        ESP_LOGE(TAG, "Write Failure: File not correctly initialized.");
        return false;
    }

    if (!file.open)
    {
        ESP_LOGE(TAG, "Write Failure: File not open.");
        return false;
    }

    fprintf(file.stream, data);

    return true;
}

bool SDLogger::write_line(File& file, const char* line)
{
    size_t line_length;
    size_t temp_buffer_sz;
    char* temp_buffer;

    if (!usability_check("Write Line Failure"))
        return false;

    if (!file.initialized)
    {
        ESP_LOGE(TAG, "Write Line Failure: File not correctly initialized.");
        return false;
    }

    if (!file.open)
    {
        ESP_LOGE(TAG, "Write Line Failure: File not open.");
        return false;
    }

    line_length = strlen(line);
    temp_buffer_sz = line_length + 2;
    temp_buffer = new char[temp_buffer_sz];

    if (temp_buffer == nullptr)
    {
        ESP_LOGE(TAG, "Write Line Failure: No heap memory available for line buffer.");
        return false;
    }

    strncpy(temp_buffer, line, line_length);

    temp_buffer[line_length] = '\n';
    temp_buffer[line_length + 1] = '\0';

    fprintf(file.stream, temp_buffer);

    delete[] temp_buffer;

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

bool SDLogger::load_sd_info()
{
    size_t buffer_sz = 1024;
    char* buffer = static_cast<char*>(malloc(buffer_sz));
    FILE* memstream;

    if (buffer == nullptr)
    {
        ESP_LOGE(TAG, "Load Sd Info Failure: No heap memory available for info buffer.");
        return false;
    }

    memstream = fmemopen(buffer, buffer_sz, "w");
    if (memstream == nullptr)
    {
        ESP_LOGW(TAG, "Load Sd Info Failure: Failed to open memory stream.");
        free(buffer);
        return false;
    }

    sdmmc_card_print_info(memstream, &card);
    fclose(memstream);

    // parse info
    if (!parse_info(buffer))
    {
        ESP_LOGW(TAG, "Load Sd Info: Failed to parse info buffer.");
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

SDLogger::File::File()
    : initialized(false)
    , open(false)
    , stream(nullptr)
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
    // reset fields if re-initializing
    initialized = false;
    open = false;

    initialized = path_parse(path);

    return initialized;
}

bool SDLogger::File::path_forbidden_char_check(const char* path)
{
    std::array<char, 8> forbidden_chars = {'\\', ':', '*', '?', '"', '<', '>', '|'};
    uint8_t period_count = 0;

    for (size_t i = 0; i < strlen(path); i++)
    {
        if (path[i] == '.')
            period_count++;

        if (period_count > 1)
        {
            ESP_LOGE(TAG, "File Initialization Failure: Invalid path, multiple '.' characters in path name.");
            return true;
        }

        for (const char forbidden_char : forbidden_chars)
            if (path[i] == forbidden_char)
            {
                ESP_LOGE(TAG, "File Initialization Failure: Invalid path, forbidden characters in path name.");
                return true;
            }
    }

    return false;
}

bool SDLogger::File::path_part_period_check(const char* part)
{
    uint8_t period_count = 0;

    for (size_t i = 0; i < strlen(part); i++)
    {
        if (part[i] == '.')
            return true;
    }

    return false;
}

bool SDLogger::File::create_path(const char* path)
{
    size_t length = strlen(path);

    if (path_forbidden_char_check(path))
        return false;

    this->path = new char[length + 1];
    if (this->path == nullptr)
    {
        ESP_LOGE(TAG, "File Initialization Failure: No heap memory available for path.");
        return false;
    }

    // copy full path (w/o root directory) to file path member
    strcpy(this->path, "/");
    strcat(this->path + 1, path);

    return true;
}

bool SDLogger::File::create_directory_path(char* dir_path)
{
    this->directory_path = new char[strlen(dir_path)];
    if (this->directory_path == nullptr)
    {
        ESP_LOGE(TAG, "File Initialization Failure: No heap memory available for directory path.");
        return false;
    }

    if (strcmp(dir_path, "") != 0)
        strcpy(this->directory_path, dir_path);

    return true;
}

bool SDLogger::File::path_tokenize_parts(const char* path, char* dir_path, char* file_name)
{
    const char* start = nullptr;
    const char* end = nullptr;

    start = path;

    if (strlen(path) > 0)
        while ((end = strchr(start, '/')) != nullptr)
        {
            if (!path_tokenize_part(static_cast<size_t>(end - start), dir_path, start))
                return false;

            start = end + 1;
        }

    // check for final part
    if (strlen(start) > 0)
    {
        if (!path_tokenize_part(strlen(start) + 1, file_name, start))
            return false;

        if (!path_part_period_check(file_name))
        {
            ESP_LOGE(TAG, "File Initialization Failure: Invalid path, '.' character must be final part of file path to indicate file extension.");
            return false;
        }
    }

    return true;
}

bool SDLogger::File::path_tokenize_part(const size_t part_length, char* output_path, const char* start)
{
    char* part;

    part = static_cast<char*>(malloc(part_length));

    if (part == nullptr)
    {
        ESP_LOGE(TAG, "File Initialization Failure: No heap memory available for parsing directory path.");
        return false;
    }

    part[part_length] = '\0';
    strncpy(part, start, part_length);
    strcat(output_path, "/");
    strcat(output_path, part);

    free(part);

    return true;
}

bool SDLogger::File::path_parse(const char* path)
{
    char file_name[50] = "";
    char dir_path[100] = "";

    if (!create_path(path))
        return false;

    if (!path_tokenize_parts(path, dir_path, file_name))
        return false;

    // copy directory path (w/o root directory) to file directory member
    if (!create_directory_path(dir_path))
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
