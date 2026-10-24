#include <atomic>
#include <array>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <strings.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_zigbee.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs_flash.h"

namespace {

constexpr char kTag[] = "zigbee_host";
constexpr uint8_t kJoinUntilClosed = 0xFF;
constexpr uint8_t kMaxChildren = 10;
constexpr size_t kMaxKnownDevices = 16;
constexpr size_t kDeviceNameSize = 32;
constexpr uint32_t kPersistedNamesVersion = 1;
constexpr uint16_t kTuyaClusterId = 0xEF00;
constexpr uint8_t kTuyaDataReportCommand = 0x02;
constexpr TickType_t kDisplayPollInterval = pdMS_TO_TICKS(1000);
constexpr TickType_t kDisplayMinimumRefreshInterval = pdMS_TO_TICKS(15000);
constexpr uint32_t kDefaultSensorStaleAfterMinutes = 60;
constexpr uint32_t kMaximumSensorStaleAfterMinutes = 10080;
constexpr char kDisplayTag[] = "epaper";
constexpr char kNameNamespace[] = "zb_names";
constexpr char kNameRecordsKey[] = "records";
constexpr gpio_num_t kAntennaSwitchEnablePin = GPIO_NUM_3;
constexpr gpio_num_t kAntennaSelectPin = GPIO_NUM_14;
constexpr gpio_num_t kDisplayBusyPin = GPIO_NUM_21;
constexpr gpio_num_t kDisplayResetPin = GPIO_NUM_22;
constexpr gpio_num_t kDisplayDataCommandPin = GPIO_NUM_23;
constexpr gpio_num_t kDisplayChipSelectPin = GPIO_NUM_2;
constexpr gpio_num_t kDisplayClockPin = GPIO_NUM_19;
constexpr gpio_num_t kDisplayMosiPin = GPIO_NUM_18;
constexpr uint16_t kDisplayWidth = 400;
constexpr uint16_t kDisplayHeight = 300;
constexpr size_t kDisplayBytesPerRow = kDisplayWidth / 8;
constexpr size_t kDisplayImageSize = kDisplayBytesPerRow * kDisplayHeight;
constexpr size_t kLvglPartialBufferBytes = kDisplayWidth * 10 * sizeof(uint16_t);
constexpr int64_t kBusyTimeoutUs = 60 * 1000 * 1000;
constexpr int64_t kBusyStartTimeoutUs = 5 * 1000 * 1000;

constexpr uint8_t kFullLutVcom[] = {
    0x00, 0x08, 0x00, 0x00, 0x00, 0x02, 0x60, 0x28, 0x28, 0x00, 0x00,
    0x01, 0x00, 0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0x12, 0x12, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
constexpr uint8_t kFullLutWhiteToWhite[] = {
    0x40, 0x08, 0x00, 0x00, 0x00, 0x02, 0x90, 0x28, 0x28, 0x00, 0x00,
    0x01, 0x40, 0x14, 0x00, 0x00, 0x00, 0x01, 0xA0, 0x12, 0x12, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
constexpr uint8_t kFullLutBlackToWhite[] = {
    0x40, 0x08, 0x00, 0x00, 0x00, 0x02, 0x90, 0x28, 0x28, 0x00, 0x00,
    0x01, 0x40, 0x14, 0x00, 0x00, 0x00, 0x01, 0xA0, 0x12, 0x12, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
constexpr uint8_t kFullLutWhiteToBlack[] = {
    0x80, 0x08, 0x00, 0x00, 0x00, 0x02, 0x90, 0x28, 0x28, 0x00, 0x00,
    0x01, 0x80, 0x14, 0x00, 0x00, 0x00, 0x01, 0x50, 0x12, 0x12, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
constexpr uint8_t kFullLutBlackToBlack[] = {
    0x80, 0x08, 0x00, 0x00, 0x00, 0x02, 0x90, 0x28, 0x28, 0x00, 0x00,
    0x01, 0x80, 0x14, 0x00, 0x00, 0x00, 0x01, 0x50, 0x12, 0x12, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

struct DeviceRecord {
    uint64_t ieee_address = 0;
    uint16_t short_address = EZB_NWK_ADDR_UNKNOWN;
    bool active = false;
    char name[kDeviceNameSize]{};
};

struct PersistedNameRecord {
    uint64_t ieee_address = 0;
    char name[kDeviceNameSize]{};
};

struct PersistedNames {
    uint32_t version = kPersistedNamesVersion;
    uint32_t count = 0;
    std::array<PersistedNameRecord, kMaxKnownDevices> records{};
};

using LegacyPersistedNames = std::array<DeviceRecord, kMaxKnownDevices>;
static_assert(sizeof(PersistedNameRecord) == sizeof(uint64_t) + kDeviceNameSize);
static_assert(sizeof(PersistedNames) == 2 * sizeof(uint32_t) +
                                             kMaxKnownDevices * sizeof(PersistedNameRecord));

struct SensorValues {
    uint32_t soil_moisture = 0;
    uint32_t temperature_deci_c = 0;
    uint32_t battery_percent = 0;
    uint32_t air_humidity = 0;
    bool soil_moisture_valid = false;
    bool temperature_valid = false;
    bool battery_valid = false;
    bool air_humidity_valid = false;
};

struct DisplayRow {
    char name[kDeviceNameSize]{};
    uint16_t short_address = EZB_NWK_ADDR_UNKNOWN;
    SensorValues values{};
    bool stale = false;
};

struct DisplayLabels {
    lv_obj_t* row_background[kMaxChildren]{};
    lv_obj_t* device[kMaxChildren]{};
    lv_obj_t* soil[kMaxChildren]{};
    lv_obj_t* air[kMaxChildren]{};
    lv_obj_t* temperature[kMaxChildren]{};
    lv_obj_t* battery[kMaxChildren]{};
    lv_obj_t* empty_status = nullptr;
};

spi_device_handle_t s_display_spi = nullptr;
alignas(4) std::array<uint8_t, kDisplayImageSize> s_display_image{};
alignas(4) std::array<uint8_t, kDisplayImageSize> s_previous_display_image{};
alignas(4) std::array<uint8_t, kLvglPartialBufferBytes> s_lvgl_partial_buffer{};
bool s_display_io_failed = false;

std::atomic<bool> s_network_ready{false};
portMUX_TYPE s_devices_mux = portMUX_INITIALIZER_UNLOCKED;
std::array<DeviceRecord, kMaxChildren> s_devices{};
std::array<SensorValues, kMaxChildren> s_sensor_values{};
std::array<int64_t, kMaxChildren> s_last_seen_us{};
std::array<int8_t, kMaxChildren> s_rssi{};
std::array<bool, kMaxChildren> s_rssi_valid{};
std::array<PersistedNameRecord, kMaxKnownDevices> s_persisted_names{};
std::atomic<uint32_t> s_sensor_stale_after_minutes{kDefaultSensorStaleAfterMinutes};

enum class RenameStage {
    None,
    EnterName,
    ConfirmRemove,
};

RenameStage s_rename_stage = RenameStage::None;
bool s_device_list_view = false;
std::array<uint64_t, kMaxChildren> s_selection{};
size_t s_selection_count = 0;
uint64_t s_selected_ieee_address = 0;

void select_external_antenna()
{
    gpio_config_t enable_config{};
    enable_config.pin_bit_mask = 1ULL << kAntennaSwitchEnablePin;
    enable_config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&enable_config));
    ESP_ERROR_CHECK(gpio_set_level(kAntennaSwitchEnablePin, 0));

    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_config_t select_config{};
    select_config.pin_bit_mask = 1ULL << kAntennaSelectPin;
    select_config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&select_config));
    ESP_ERROR_CHECK(gpio_set_level(kAntennaSelectPin, 1));
}

bool save_device_names();

void load_device_names()
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(kNameNamespace, NVS_READONLY, &handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    if (result != ESP_OK) {
        std::printf("Could not open device-name storage: %s\n", esp_err_to_name(result));
        return;
    }

    size_t size = 0;
    result = nvs_get_blob(handle, kNameRecordsKey, nullptr, &size);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return;
    }
    if (result != ESP_OK) {
        nvs_close(handle);
        std::printf("Could not inspect device-name storage: %s\n", esp_err_to_name(result));
        return;
    }

    auto import_name = [](uint64_t ieee_address, const char* name) {
        if (ieee_address == 0 || name[0] == '\0') {
            return;
        }
        for (PersistedNameRecord& record : s_persisted_names) {
            if (record.ieee_address == ieee_address) {
                std::snprintf(record.name, sizeof(record.name), "%s", name);
                return;
            }
            if (record.ieee_address == 0) {
                record.ieee_address = ieee_address;
                std::snprintf(record.name, sizeof(record.name), "%s", name);
                return;
            }
        }
    };

    bool migrate_legacy_names = false;
    if (size == sizeof(LegacyPersistedNames)) {
        LegacyPersistedNames legacy{};
        size_t legacy_size = sizeof(legacy);
        result = nvs_get_blob(handle, kNameRecordsKey, legacy.data(), &legacy_size);
        if (result == ESP_OK && legacy_size == sizeof(legacy)) {
            portENTER_CRITICAL(&s_devices_mux);
            for (DeviceRecord& record : legacy) {
                record.name[kDeviceNameSize - 1] = '\0';
                import_name(record.ieee_address, record.name);
            }
            portEXIT_CRITICAL(&s_devices_mux);
            migrate_legacy_names = true;
        } else if (result == ESP_OK) {
            result = ESP_ERR_INVALID_SIZE;
        }
    } else if (size == sizeof(PersistedNames)) {
        PersistedNames names{};
        size_t names_size = sizeof(names);
        result = nvs_get_blob(handle, kNameRecordsKey, &names, &names_size);
        if (result == ESP_OK && names_size == sizeof(names) &&
            names.version == kPersistedNamesVersion &&
            names.count <= names.records.size()) {
            portENTER_CRITICAL(&s_devices_mux);
            for (size_t i = 0; i < names.count; ++i) {
                names.records[i].name[kDeviceNameSize - 1] = '\0';
                import_name(names.records[i].ieee_address, names.records[i].name);
            }
            portEXIT_CRITICAL(&s_devices_mux);
        } else if (result == ESP_OK) {
            result = ESP_ERR_INVALID_SIZE;
        }
    } else {
        result = ESP_ERR_INVALID_SIZE;
    }
    nvs_close(handle);

    if (result != ESP_OK) {
        std::printf("Device-name storage is invalid or unreadable: %s\n",
                    esp_err_to_name(result));
    } else if (migrate_legacy_names) {
        (void)save_device_names();
    }
}

void load_sensor_timeout()
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(kNameNamespace, NVS_READONLY, &handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    if (result != ESP_OK) {
        std::printf("Could not open sensor-timeout storage: %s\n", esp_err_to_name(result));
        return;
    }

    uint32_t minutes = kDefaultSensorStaleAfterMinutes;
    result = nvs_get_u32(handle, "stale_minutes", &minutes);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    if (result != ESP_OK) {
        std::printf("Could not load sensor timeout: %s\n", esp_err_to_name(result));
        return;
    }
    if (minutes < 1 || minutes > kMaximumSensorStaleAfterMinutes) {
        std::printf("Stored sensor timeout is invalid; using default %u minutes.\n",
                    static_cast<unsigned>(kDefaultSensorStaleAfterMinutes));
        return;
    }

    s_sensor_stale_after_minutes.store(minutes);
}

bool save_sensor_timeout(uint32_t minutes)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(kNameNamespace, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        std::printf("Could not open sensor-timeout storage: %s\n", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_u32(handle, "stale_minutes", minutes);
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    if (result != ESP_OK) {
        std::printf("Could not save sensor timeout: %s\n", esp_err_to_name(result));
        return false;
    }
    return true;
}

bool save_device_names()
{
    PersistedNames names{};
    portENTER_CRITICAL(&s_devices_mux);
    for (const PersistedNameRecord& record : s_persisted_names) {
        if (record.ieee_address == 0 || record.name[0] == '\0') {
            continue;
        }
        names.records[names.count++] = record;
    }
    portEXIT_CRITICAL(&s_devices_mux);

    nvs_handle_t handle;
    esp_err_t result = nvs_open(kNameNamespace, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        std::printf("Could not open device-name storage: %s\n", esp_err_to_name(result));
        return false;
    }
    result = nvs_set_blob(handle, kNameRecordsKey, &names, sizeof(names));
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    if (result != ESP_OK) {
        std::printf("Could not save device names: %s\n", esp_err_to_name(result));
        return false;
    }
    return true;
}

size_t find_or_add_device(uint64_t ieee_address, uint16_t short_address, bool active)
{
    if (ieee_address == 0) {
        return s_devices.size();
    }

    size_t free_slot = s_devices.size();
    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < s_devices.size(); ++i) {
        if (s_devices[i].active && s_devices[i].ieee_address == ieee_address) {
            s_devices[i].short_address = short_address;
            s_devices[i].active = active;
            portEXIT_CRITICAL(&s_devices_mux);
            return i;
        }
        if (free_slot == s_devices.size() && !s_devices[i].active) {
            free_slot = i;
        }
    }
    if (active && free_slot < s_devices.size()) {
        s_devices[free_slot].ieee_address = ieee_address;
        s_devices[free_slot].short_address = short_address;
        s_devices[free_slot].active = active;
        s_sensor_values[free_slot] = SensorValues{};
        s_last_seen_us[free_slot] = 0;
        s_rssi[free_slot] = 0;
        s_rssi_valid[free_slot] = false;
        for (const PersistedNameRecord& record : s_persisted_names) {
            if (record.ieee_address == ieee_address) {
                std::snprintf(s_devices[free_slot].name,
                              sizeof(s_devices[free_slot].name), "%s", record.name);
                break;
            }
        }
    }
    portEXIT_CRITICAL(&s_devices_mux);
    return free_slot;
}

void update_device_rssi(uint64_t ieee_address, int8_t rssi)
{
    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < s_devices.size(); ++i) {
        if (s_devices[i].active && s_devices[i].ieee_address == ieee_address) {
            s_rssi[i] = rssi;
            s_rssi_valid[i] = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_devices_mux);
}

void get_device_label(uint16_t short_address, char* label, size_t label_size)
{
    label[0] = '\0';
    portENTER_CRITICAL(&s_devices_mux);
    for (const DeviceRecord& device : s_devices) {
        if (device.active && device.short_address == short_address) {
            std::snprintf(label, label_size, "%s", device.name);
            break;
        }
    }
    portEXIT_CRITICAL(&s_devices_mux);
    if (label[0] == '\0') {
        std::snprintf(label, label_size, "Unnamed device");
    }
}

void refresh_neighbors()
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_nwk_info_iterator_t iterator = EZB_NWK_INFO_ITERATOR_INIT;
    ezb_nwk_neighbor_info_t neighbor{};
    while (ezb_nwk_get_next_neighbor(&iterator, &neighbor) == EZB_ERR_NONE) {
        if (find_or_add_device(neighbor.ieee_addr.u64, neighbor.short_addr, true) <
            s_devices.size()) {
            update_device_rssi(neighbor.ieee_addr.u64, neighbor.rssi);
        }
    }
    esp_zigbee_lock_release();
}

void device_refresh_task(void*)
{
    while (!s_network_ready.load()) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        if (attempt > 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        refresh_neighbors();
    }

    vTaskDelete(nullptr);
}

void store_sensor_value(uint16_t short_address, uint8_t datapoint_id, uint32_t value)
{
    bool stored = false;
    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < s_devices.size(); ++i) {
        const DeviceRecord& device = s_devices[i];
        if (!device.active || device.short_address != short_address) {
            continue;
        }

        SensorValues& values = s_sensor_values[i];
        switch (datapoint_id) {
        case 3:
            values.soil_moisture = value;
            values.soil_moisture_valid = true;
            stored = true;
            break;
        case 5:
            values.temperature_deci_c = value;
            values.temperature_valid = true;
            stored = true;
            break;
        case 15:
            values.battery_percent = value;
            values.battery_valid = true;
            stored = true;
            break;
        case 109:
            values.air_humidity = value;
            values.air_humidity_valid = true;
            stored = true;
            break;
        default:
            break;
        }
        if (stored) {
            s_last_seen_us[i] = esp_timer_get_time();
        }
        break;
    }
    portEXIT_CRITICAL(&s_devices_mux);

    if (!stored && (datapoint_id == 3 || datapoint_id == 5 ||
                    datapoint_id == 15 || datapoint_id == 109)) {
        ESP_LOGW(kDisplayTag, "DP%u value %lu from short address 0x%04x not stored: device is not active",
                 datapoint_id, static_cast<unsigned long>(value), short_address);
    } else if (stored) {
        ESP_LOGI(kDisplayTag, "Stored latest DP%u value %lu for short address 0x%04x",
                 datapoint_id, static_cast<unsigned long>(value), short_address);
    }
}

size_t get_display_snapshot(std::array<DisplayRow, kMaxChildren>& rows)
{
    size_t count = 0;
    const int64_t now = esp_timer_get_time();
    const int64_t stale_after_us =
        static_cast<int64_t>(s_sensor_stale_after_minutes.load()) * 60 * 1000 * 1000;
    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < s_devices.size() && count < rows.size(); ++i) {
        if (!s_devices[i].active) {
            continue;
        }

        DisplayRow& row = rows[count++];
        std::memcpy(row.name, s_devices[i].name, sizeof(row.name));
        row.name[sizeof(row.name) - 1] = '\0';
        row.short_address = s_devices[i].short_address;
        row.values = s_sensor_values[i];
        row.stale = s_last_seen_us[i] > 0 &&
                    now - s_last_seen_us[i] >= stale_after_us;
    }
    portEXIT_CRITICAL(&s_devices_mux);

    for (size_t i = 0; i < count; ++i) {
        if (rows[i].name[0] == '\0') {
            std::snprintf(rows[i].name, sizeof(rows[i].name), "Sensor %04X",
                          rows[i].short_address);
        }
    }
    return count;
}

bool display_snapshots_equal(const std::array<DisplayRow, kMaxChildren>& lhs,
                             size_t lhs_count,
                             const std::array<DisplayRow, kMaxChildren>& rhs,
                             size_t rhs_count)
{
    if (lhs_count != rhs_count) {
        return false;
    }

    for (size_t i = 0; i < lhs_count; ++i) {
        const SensorValues& a = lhs[i].values;
        const SensorValues& b = rhs[i].values;
        if (lhs[i].short_address != rhs[i].short_address ||
            std::strcmp(lhs[i].name, rhs[i].name) != 0 ||
            lhs[i].stale != rhs[i].stale ||
            a.soil_moisture != b.soil_moisture ||
            a.temperature_deci_c != b.temperature_deci_c ||
            a.battery_percent != b.battery_percent ||
            a.air_humidity != b.air_humidity ||
            a.soil_moisture_valid != b.soil_moisture_valid ||
            a.temperature_valid != b.temperature_valid ||
            a.battery_valid != b.battery_valid ||
            a.air_humidity_valid != b.air_humidity_valid) {
            return false;
        }
    }
    return true;
}

esp_err_t display_transmit(const uint8_t* bytes, size_t length)
{
    spi_transaction_t transaction{};
    transaction.length = length * 8;
    transaction.tx_buffer = bytes;
    const esp_err_t result = spi_device_transmit(s_display_spi, &transaction);
    if (result != ESP_OK) {
        ESP_LOGE(kDisplayTag, "SPI transfer of %u bytes failed: %s",
                 static_cast<unsigned>(length), esp_err_to_name(result));
    }
    return result;
}

esp_err_t display_send_command(uint8_t command)
{
    esp_err_t result = gpio_set_level(kDisplayDataCommandPin, 0);
    if (result != ESP_OK) {
        ESP_LOGE(kDisplayTag, "Could not select command mode: %s", esp_err_to_name(result));
        return result;
    }
    return display_transmit(&command, sizeof(command));
}

esp_err_t display_send_data(const uint8_t* data, size_t length)
{
    esp_err_t result = gpio_set_level(kDisplayDataCommandPin, 1);
    if (result != ESP_OK) {
        ESP_LOGE(kDisplayTag, "Could not select data mode: %s", esp_err_to_name(result));
        return result;
    }
    return display_transmit(data, length);
}

esp_err_t display_send_register(uint8_t command, const uint8_t* data, size_t length)
{
    esp_err_t result = display_send_command(command);
    return result == ESP_OK ? display_send_data(data, length) : result;
}

esp_err_t display_wait_idle(const char* stage)
{
    const int64_t start_time = esp_timer_get_time();
    int64_t last_report_time = start_time;
    ESP_LOGI(kDisplayTag, "%s: waiting for BUSY HIGH (idle), current=%d",
             stage, gpio_get_level(kDisplayBusyPin));
    while (gpio_get_level(kDisplayBusyPin) == 0) {
        const int64_t now = esp_timer_get_time();
        if (now - start_time > kBusyTimeoutUs) {
            ESP_LOGE(kDisplayTag, "%s: timed out waiting for BUSY GPIO%d to go HIGH",
                     stage, kDisplayBusyPin);
            return ESP_ERR_TIMEOUT;
        }
        if (now - last_report_time >= 1000000) {
            ESP_LOGI(kDisplayTag, "%s: still busy (LOW)", stage);
            last_report_time = now;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(kDisplayTag, "%s: display idle after %lld ms",
             stage, (esp_timer_get_time() - start_time) / 1000);
    return ESP_OK;
}

esp_err_t display_wait_busy_start()
{
    const int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(kDisplayBusyPin) != 0) {
        const int64_t now = esp_timer_get_time();
        if (now - start_time > kBusyStartTimeoutUs) {
            ESP_LOGE(kDisplayTag, "Refresh never asserted BUSY LOW");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(kDisplayTag, "Panel asserted BUSY LOW after %lld ms",
             (esp_timer_get_time() - start_time) / 1000);
    return ESP_OK;
}

esp_err_t display_initialize_controller()
{
    ESP_LOGI(kDisplayTag,
             "Initializing Waveshare 4.2-inch V2 / IL0398 display: BUSY=D3/GPIO%d "
             "RST=D4/GPIO%d DC=D5/GPIO%d CS=D2/GPIO%d CLK=D8/GPIO%d DIN=D10/GPIO%d",
             kDisplayBusyPin, kDisplayResetPin, kDisplayDataCommandPin,
             kDisplayChipSelectPin, kDisplayClockPin, kDisplayMosiPin);

    gpio_config_t output_config{};
    output_config.pin_bit_mask =
        (1ULL << kDisplayResetPin) | (1ULL << kDisplayDataCommandPin);
    output_config.mode = GPIO_MODE_OUTPUT;
    esp_err_t result = gpio_config(&output_config);
    if (result != ESP_OK) {
        ESP_LOGE(kDisplayTag, "Could not configure display output GPIOs: %s",
                 esp_err_to_name(result));
        return result;
    }

    gpio_config_t busy_config{};
    busy_config.pin_bit_mask = 1ULL << kDisplayBusyPin;
    busy_config.mode = GPIO_MODE_INPUT;
    busy_config.pull_up_en = GPIO_PULLUP_DISABLE;
    busy_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    result = gpio_config(&busy_config);
    if (result != ESP_OK) {
        ESP_LOGE(kDisplayTag, "Could not configure BUSY GPIO: %s", esp_err_to_name(result));
        return result;
    }

    spi_bus_config_t bus_config{};
    bus_config.mosi_io_num = kDisplayMosiPin;
    bus_config.miso_io_num = -1;
    bus_config.sclk_io_num = kDisplayClockPin;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = kDisplayImageSize;
    result = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (result != ESP_OK) {
        ESP_LOGE(kDisplayTag, "Could not initialize display SPI bus: %s",
                 esp_err_to_name(result));
        return result;
    }

    spi_device_interface_config_t device_config{};
    device_config.clock_speed_hz = 2 * 1000 * 1000;
    device_config.mode = 0;
    device_config.spics_io_num = kDisplayChipSelectPin;
    device_config.queue_size = 1;
    result = spi_bus_add_device(SPI2_HOST, &device_config, &s_display_spi);
    if (result != ESP_OK) {
        ESP_LOGE(kDisplayTag, "Could not add display SPI device: %s",
                 esp_err_to_name(result));
        return result;
    }
    ESP_LOGI(kDisplayTag, "Display SPI ready at 2 MHz; BUSY initially reads %d",
             gpio_get_level(kDisplayBusyPin));

    ESP_LOGI(kDisplayTag, "Resetting e-paper controller");
    result = gpio_set_level(kDisplayResetPin, 1);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(100));
    result = gpio_set_level(kDisplayResetPin, 0);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(2));
    result = gpio_set_level(kDisplayResetPin, 1);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(100));
    result = display_wait_idle("after hardware reset");
    if (result != ESP_OK) return result;

    const uint8_t power_settings[] = {0x03, 0x00, 0x2B, 0x2B};
    const uint8_t booster_settings[] = {0x17, 0x17, 0x17};
    const uint8_t panel_settings[] = {0x3F};
    const uint8_t pll_settings[] = {0x3A};
    const uint8_t resolution[] = {0x01, 0x90, 0x01, 0x2C};
    const uint8_t vcom_dc[] = {0x12};
    const uint8_t vcom_interval[] = {0xD7};
    const auto send = [](uint8_t command, const uint8_t* bytes, size_t length,
                         const char* operation) {
        const esp_err_t send_result = display_send_register(command, bytes, length);
        if (send_result != ESP_OK) {
            ESP_LOGE(kDisplayTag, "Could not %s: %s",
                     operation, esp_err_to_name(send_result));
        }
        return send_result;
    };

    ESP_LOGI(kDisplayTag, "Configuring IL0398 power and panel registers");
    if ((result = send(0x01, power_settings, sizeof(power_settings), "set panel power")) != ESP_OK ||
        (result = send(0x06, booster_settings, sizeof(booster_settings), "set booster")) != ESP_OK ||
        (result = send(0x00, panel_settings, sizeof(panel_settings), "set panel mode")) != ESP_OK ||
        (result = send(0x30, pll_settings, sizeof(pll_settings), "set panel PLL")) != ESP_OK ||
        (result = send(0x61, resolution, sizeof(resolution), "set panel resolution")) != ESP_OK ||
        (result = send(0x82, vcom_dc, sizeof(vcom_dc), "set VCOM")) != ESP_OK ||
        (result = send(0x50, vcom_interval, sizeof(vcom_interval), "set VCOM interval")) != ESP_OK) {
        return result;
    }

    const struct {
        uint8_t command;
        const uint8_t* data;
        size_t size;
    } waveforms[] = {
        {0x20, kFullLutVcom, sizeof(kFullLutVcom)},
        {0x21, kFullLutWhiteToWhite, sizeof(kFullLutWhiteToWhite)},
        {0x22, kFullLutBlackToWhite, sizeof(kFullLutBlackToWhite)},
        {0x23, kFullLutWhiteToBlack, sizeof(kFullLutWhiteToBlack)},
        {0x24, kFullLutBlackToBlack, sizeof(kFullLutBlackToBlack)},
    };
    for (const auto& waveform : waveforms) {
        result = display_send_register(waveform.command, waveform.data, waveform.size);
        if (result != ESP_OK) {
            ESP_LOGE(kDisplayTag, "Could not load waveform register 0x%02x: %s",
                     waveform.command, esp_err_to_name(result));
            return result;
        }
    }

    ESP_LOGI(kDisplayTag, "Powering on controller");
    result = display_send_command(0x04);
    if (result != ESP_OK) return result;
    result = display_wait_idle("after controller power-on");
    if (result != ESP_OK) return result;
    ESP_LOGI(kDisplayTag, "IL0398 initialization complete");
    return ESP_OK;
}

esp_err_t display_refresh()
{
    ESP_LOGI(kDisplayTag, "Sending previous and current 1-bit framebuffers");
    esp_err_t result = display_send_command(0x10);
    if (result == ESP_OK) {
        result = display_send_data(s_previous_display_image.data(), kDisplayImageSize);
    }
    if (result == ESP_OK) {
        result = display_send_command(0x13);
    }
    if (result == ESP_OK) {
        result = display_send_data(s_display_image.data(), kDisplayImageSize);
    }
    if (result == ESP_OK) {
        ESP_LOGI(kDisplayTag, "Starting full panel refresh");
        result = display_send_command(0x12);
    }
    if (result == ESP_OK) {
        result = display_wait_busy_start();
    }
    if (result == ESP_OK) {
        result = display_wait_idle("after full refresh");
    }
    if (result == ESP_OK) {
        s_previous_display_image = s_display_image;
        ESP_LOGI(kDisplayTag, "Full panel refresh completed");
    }
    return result;
}

void display_lvgl_flush(lv_display_t* display, const lv_area_t* area, uint8_t* color_data)
{
    const auto* pixels = reinterpret_cast<const uint16_t*>(color_data);
    for (int y = area->y1; y <= area->y2; ++y) {
        for (int x = area->x1; x <= area->x2; ++x) {
            const uint16_t color = *pixels++;
            const uint8_t red = (color >> 11) & 0x1F;
            const uint8_t green = (color >> 5) & 0x3F;
            const uint8_t blue = color & 0x1F;
            const unsigned luminance = (red * 255U / 31U * 299U +
                                        green * 255U / 63U * 587U +
                                        blue * 255U / 31U * 114U) / 1000U;
            const size_t index = static_cast<size_t>(y) * kDisplayBytesPerRow +
                                 static_cast<size_t>(x) / 8;
            const uint8_t mask = static_cast<uint8_t>(0x80U >> (x % 8));
            if (luminance < 128) {
                s_display_image[index] &= static_cast<uint8_t>(~mask);
            } else {
                s_display_image[index] |= mask;
            }
        }
    }

    if (area->x2 >= kDisplayWidth - 1 && area->y2 >= kDisplayHeight - 1) {
        const esp_err_t result = display_refresh();
        if (result != ESP_OK) {
            s_display_io_failed = true;
            ESP_LOGE(kDisplayTag, "Display refresh failed: %s", esp_err_to_name(result));
        }
    }
    lv_display_flush_ready(display);
}

lv_obj_t* create_display_label(lv_obj_t* parent, int x, int y, int width)
{
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_size(label, width, 18);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    return label;
}

void create_display_ui(DisplayLabels& labels)
{
    lv_obj_t* screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen, lv_color_black(), LV_PART_MAIN);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(screen);
    lv_label_set_text(title, "Plant sensor readings");
    lv_obj_set_width(title, kDisplayWidth);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

    const int column_y = 38;
    lv_obj_t* header = create_display_label(screen, 8, column_y, 145);
    lv_label_set_text(header, "Device");
    header = create_display_label(screen, 165, column_y, 48);
    lv_label_set_text(header, "Soil");
    header = create_display_label(screen, 218, column_y, 48);
    lv_label_set_text(header, "Air");
    header = create_display_label(screen, 271, column_y, 60);
    lv_label_set_text(header, "Temp");
    header = create_display_label(screen, 337, column_y, 55);
    lv_label_set_text(header, "Batt");

    lv_obj_t* divider = lv_obj_create(screen);
    lv_obj_set_pos(divider, 8, 57);
    lv_obj_set_size(divider, 384, 1);
    lv_obj_set_style_bg_color(divider, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(divider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(divider, 0, LV_PART_MAIN);
    lv_obj_remove_flag(divider, LV_OBJ_FLAG_SCROLLABLE);

    for (size_t i = 0; i < kMaxChildren; ++i) {
        const int y = 63 + static_cast<int>(i) * 20;
        labels.row_background[i] = lv_obj_create(screen);
        lv_obj_set_pos(labels.row_background[i], 8, y);
        lv_obj_set_size(labels.row_background[i], 384, 18);
        lv_obj_set_style_bg_color(labels.row_background[i], lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(labels.row_background[i], LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(labels.row_background[i], 0, LV_PART_MAIN);
        lv_obj_set_style_radius(labels.row_background[i], 0, LV_PART_MAIN);
        lv_obj_remove_flag(labels.row_background[i], LV_OBJ_FLAG_SCROLLABLE);
        labels.device[i] = create_display_label(screen, 8, y, 145);
        labels.soil[i] = create_display_label(screen, 165, y, 48);
        labels.air[i] = create_display_label(screen, 218, y, 48);
        labels.temperature[i] = create_display_label(screen, 271, y, 60);
        labels.battery[i] = create_display_label(screen, 337, y, 55);
    }

    labels.empty_status = lv_label_create(screen);
    lv_label_set_text(labels.empty_status, "Waiting for joined sensors");
    lv_obj_set_pos(labels.empty_status, 8, 110);
}

void render_display_snapshot(const DisplayLabels& labels,
                             const std::array<DisplayRow, kMaxChildren>& rows,
                             size_t count)
{
    ESP_LOGI(kDisplayTag, "Rendering %u active sensor row(s)", static_cast<unsigned>(count));
    lv_label_set_text(labels.empty_status, count == 0 ? "Waiting for joined sensors" : "");
    for (size_t i = 0; i < kMaxChildren; ++i) {
        if (i >= count) {
            lv_obj_set_style_bg_opa(labels.row_background[i], LV_OPA_TRANSP, LV_PART_MAIN);
            lv_label_set_text(labels.device[i], "");
            lv_label_set_text(labels.soil[i], "");
            lv_label_set_text(labels.air[i], "");
            lv_label_set_text(labels.temperature[i], "");
            lv_label_set_text(labels.battery[i], "");
            continue;
        }

        const DisplayRow& row = rows[i];
        const lv_color_t text_color = row.stale ? lv_color_white() : lv_color_black();
        lv_obj_set_style_bg_opa(labels.row_background[i],
                                row.stale ? LV_OPA_COVER : LV_OPA_TRANSP,
                                LV_PART_MAIN);
        lv_obj_set_style_text_color(labels.device[i], text_color, LV_PART_MAIN);
        lv_obj_set_style_text_color(labels.soil[i], text_color, LV_PART_MAIN);
        lv_obj_set_style_text_color(labels.air[i], text_color, LV_PART_MAIN);
        lv_obj_set_style_text_color(labels.temperature[i], text_color, LV_PART_MAIN);
        lv_obj_set_style_text_color(labels.battery[i], text_color, LV_PART_MAIN);
        char soil[16]{};
        char air[16]{};
        char temperature[32]{};
        char battery[16]{};
        if (row.values.soil_moisture_valid) {
            std::snprintf(soil, sizeof(soil), "%lu%%",
                          static_cast<unsigned long>(row.values.soil_moisture));
        } else {
            std::snprintf(soil, sizeof(soil), "--");
        }
        if (row.values.air_humidity_valid) {
            std::snprintf(air, sizeof(air), "%lu%%",
                          static_cast<unsigned long>(row.values.air_humidity));
        } else {
            std::snprintf(air, sizeof(air), "--");
        }
        if (row.values.temperature_valid) {
            std::snprintf(temperature, sizeof(temperature), "%lu.%luC",
                          static_cast<unsigned long>(row.values.temperature_deci_c / 10),
                          static_cast<unsigned long>(row.values.temperature_deci_c % 10));
        } else {
            std::snprintf(temperature, sizeof(temperature), "--");
        }
        if (row.values.battery_valid) {
            std::snprintf(battery, sizeof(battery), "%lu%%",
                          static_cast<unsigned long>(row.values.battery_percent));
        } else {
            std::snprintf(battery, sizeof(battery), "--");
        }
        ESP_LOGI(kDisplayTag,
                 "Row %u \"%s\" (0x%04x): soil=%s air=%s temp=%s battery=%s",
                 static_cast<unsigned>(i + 1), row.name, row.short_address,
                 soil, air, temperature, battery);
        lv_label_set_text(labels.device[i], row.name);
        if (row.values.soil_moisture_valid) {
            lv_label_set_text_fmt(labels.soil[i], "%lu%%",
                                  static_cast<unsigned long>(row.values.soil_moisture));
        } else {
            lv_label_set_text(labels.soil[i], "--");
        }
        if (row.values.air_humidity_valid) {
            lv_label_set_text_fmt(labels.air[i], "%lu%%",
                                  static_cast<unsigned long>(row.values.air_humidity));
        } else {
            lv_label_set_text(labels.air[i], "--");
        }
        if (row.values.temperature_valid) {
            lv_label_set_text_fmt(labels.temperature[i], "%lu.%luC",
                                  static_cast<unsigned long>(row.values.temperature_deci_c / 10),
                                  static_cast<unsigned long>(row.values.temperature_deci_c % 10));
        } else {
            lv_label_set_text(labels.temperature[i], "--");
        }
        if (row.values.battery_valid) {
            lv_label_set_text_fmt(labels.battery[i], "%lu%%",
                                  static_cast<unsigned long>(row.values.battery_percent));
        } else {
            lv_label_set_text(labels.battery[i], "--");
        }
    }
}

void epaper_display_task(void*)
{
    ESP_LOGI(kDisplayTag, "Display task started on core %d", xPortGetCoreID());
    lv_init();
    s_display_image.fill(0xFF);
    s_previous_display_image.fill(0xFF);
    s_display_io_failed = false;

    esp_err_t result = display_initialize_controller();
    if (result != ESP_OK) {
        ESP_LOGE(kDisplayTag, "Display initialization failed: %s", esp_err_to_name(result));
        vTaskDelete(nullptr);
        return;
    }

    lv_display_t* display = lv_display_create(kDisplayWidth, kDisplayHeight);
    if (display == nullptr) {
        ESP_LOGE(kDisplayTag, "Could not create LVGL display");
        vTaskDelete(nullptr);
        return;
    }
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(display, display_lvgl_flush);
    lv_display_set_buffers(display, s_lvgl_partial_buffer.data(), nullptr,
                           s_lvgl_partial_buffer.size(), LV_DISPLAY_RENDER_MODE_PARTIAL);
    ESP_LOGI(kDisplayTag, "LVGL display ready using 10-line RGB565 render buffer");

    DisplayLabels labels{};
    create_display_ui(labels);
    std::array<DisplayRow, kMaxChildren> previous_rows{};
    size_t previous_count = get_display_snapshot(previous_rows);
    render_display_snapshot(labels, previous_rows, previous_count);
    ESP_LOGI(kDisplayTag, "Rendering initial sensor dashboard");
    lv_obj_invalidate(lv_screen_active());
    lv_refr_now(display);
    if (s_display_io_failed) {
        ESP_LOGE(kDisplayTag, "Initial e-paper refresh failed; display task stopping");
        vTaskDelete(nullptr);
        return;
    }
    TickType_t last_refresh = xTaskGetTickCount();
    ESP_LOGI(kDisplayTag, "Display ready; polling sensor values every %lu ms",
             static_cast<unsigned long>(kDisplayPollInterval * portTICK_PERIOD_MS));

    while (true) {
        vTaskDelay(kDisplayPollInterval);
        std::array<DisplayRow, kMaxChildren> current_rows{};
        const size_t current_count = get_display_snapshot(current_rows);
        const TickType_t now = xTaskGetTickCount();
        if (!display_snapshots_equal(previous_rows, previous_count, current_rows, current_count) &&
            now - last_refresh >= kDisplayMinimumRefreshInterval) {
            render_display_snapshot(labels, current_rows, current_count);
            ESP_LOGI(kDisplayTag, "Refreshing full e-paper dashboard");
            lv_obj_invalidate(lv_screen_active());
            lv_refr_now(display);
            if (s_display_io_failed) {
                ESP_LOGE(kDisplayTag, "Dashboard refresh failed; display task stopping");
                vTaskDelete(nullptr);
                return;
            }
            previous_rows = current_rows;
            previous_count = current_count;
            last_refresh = xTaskGetTickCount();
        } else if (!display_snapshots_equal(previous_rows, previous_count, current_rows, current_count)) {
            ESP_LOGD(kDisplayTag, "Sensor values changed; waiting for refresh interval");
        }
    }
}

bool handle_tuya_data_report(const ezb_zcl_raw_frame_t* raw_frame)
{
    if (raw_frame == nullptr || raw_frame->header == nullptr ||
        raw_frame->header->cluster_id != kTuyaClusterId ||
        raw_frame->header->cmd_id != kTuyaDataReportCommand) {
        return false;
    }

    if (raw_frame->payload == nullptr || raw_frame->payload_length < 2) {
        ESP_LOGW(kTag, "Ignoring malformed Tuya datapoint report");
        return true;
    }

    uint16_t source_short_address = EZB_NWK_ADDR_UNKNOWN;
    if (ezb_address_is_short(&raw_frame->header->src_addr,
                             raw_frame->header->src_addr.u.short_addr)) {
        source_short_address = raw_frame->header->src_addr.u.short_addr;
    }
    char device_name[kDeviceNameSize]{};
    get_device_label(source_short_address, device_name, sizeof(device_name));

    size_t offset = 2;
    while (offset < raw_frame->payload_length) {
        if (raw_frame->payload_length - offset < 4) {
            ESP_LOGW(kTag, "Ignoring truncated Tuya datapoint header");
            return true;
        }

        const uint8_t datapoint_id = raw_frame->payload[offset];
        const uint8_t datapoint_type = raw_frame->payload[offset + 1];
        const size_t value_length =
            (static_cast<size_t>(raw_frame->payload[offset + 2]) << 8) |
            raw_frame->payload[offset + 3];
        offset += 4;
        if (value_length > raw_frame->payload_length - offset) {
            ESP_LOGW(kTag, "Ignoring truncated Tuya datapoint %u", datapoint_id);
            return true;
        }
       
        if (datapoint_type == 0x02 && value_length == 4) {
            const uint8_t* value = &raw_frame->payload[offset];
            const uint32_t number = (static_cast<uint32_t>(value[0]) << 24) |
                                    (static_cast<uint32_t>(value[1]) << 16) |
                                    (static_cast<uint32_t>(value[2]) << 8) |
                                    static_cast<uint32_t>(value[3]);
            store_sensor_value(source_short_address, datapoint_id, number);
            // ESP_LOGI(kTag,"----------------------------------------");
            // ESP_LOGI(kTag, "Tuya datapoint %u (type %u) with %zu bytes received from %s (0x%04x) Value: %lu",
            //         datapoint_id, datapoint_type, value_length, device_name, source_short_address, static_cast<unsigned long>(number));
            switch (datapoint_id) {
            case 3:
                ESP_LOGI(kTag, "%s (0x%04x) soil moisture: %lu %%", device_name,
                         source_short_address, static_cast<unsigned long>(number));
                break;
            case 5:
                ESP_LOGI(kTag, "%s (0x%04x) temperature: %.1f C", device_name,
                         source_short_address, static_cast<double>(number) / 10.0);
                break;
            case 15:
                ESP_LOGI(kTag, "%s (0x%04x) battery: %lu %%", device_name,
                         source_short_address, static_cast<unsigned long>(number));
                break;
            case 109:
                ESP_LOGI(kTag, "%s (0x%04x) air humidity: %lu %%", device_name,
                         source_short_address, static_cast<unsigned long>(number));
                break;
            default:
                break;
            }
        }
        offset += value_length;
    }
    return true;
}

bool zigbee_raw_frame_handler(const ezb_zcl_raw_frame_t* raw_frame)
{
    return handle_tuya_data_report(raw_frame);
}

bool zigbee_signal_handler(const ezb_app_signal_t* app_signal)
{
    const ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(kTag, "Initializing Zigbee commissioning");
        (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        const auto* status = static_cast<const ezb_bdb_comm_status_t*>(
            ezb_app_signal_get_params(app_signal));
        if (status == nullptr || *status != EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGE(kTag, "Zigbee startup failed");
            break;
        }

        if (ezb_bdb_is_factory_new()) {
            ESP_LOGI(kTag, "Forming a new Zigbee network");
            (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            s_network_ready.store(true);
            ESP_LOGI(kTag, "Coordinator rejoined its existing Zigbee network");
        }
        break;
    }

    case EZB_BDB_SIGNAL_FORMATION: {
        const auto* status = static_cast<const ezb_bdb_comm_status_t*>(
            ezb_app_signal_get_params(app_signal));
        if (status != nullptr && *status == EZB_BDB_STATUS_SUCCESS) {
            s_network_ready.store(true);
            ESP_LOGI(kTag, "Zigbee network formed (PAN ID 0x%04hx, channel %u)",
                     ezb_nwk_get_panid(), ezb_nwk_get_current_channel());
        } else {
            ESP_LOGE(kTag, "Zigbee network formation failed");
        }
        break;
    }

    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const auto* device = static_cast<const ezb_zdo_signal_device_annce_params_t*>(
            ezb_app_signal_get_params(app_signal));
        if (device != nullptr) {
            if (find_or_add_device(device->device_addr.u64, device->short_addr, true) ==
                s_devices.size()) {
                ESP_LOGW(kTag, "Device list is full; could not track 0x%016llx",
                         static_cast<unsigned long long>(device->device_addr.u64));
            }
            ESP_LOGI(kTag, "Device joined or rejoined (short address 0x%04hx)",
                     device->short_addr);
        }
        break;
    }

    case EZB_ZDO_SIGNAL_LEAVE_INDICATION: {
        const auto* device = static_cast<const ezb_zdo_signal_leave_indication_params_t*>(
            ezb_app_signal_get_params(app_signal));
        if (device != nullptr) {
            portENTER_CRITICAL(&s_devices_mux);
            for (size_t i = 0; i < s_devices.size(); ++i) {
                if (s_devices[i].ieee_address == device->device_addr.u64) {
                    s_devices[i] = DeviceRecord{};
                    s_sensor_values[i] = SensorValues{};
                    s_last_seen_us[i] = 0;
                    s_rssi[i] = 0;
                    s_rssi_valid[i] = false;
                    break;
                }
            }
            portEXIT_CRITICAL(&s_devices_mux);
            ESP_LOGI(kTag, "Device left (short address 0x%04hx)", device->short_addr);
        }
        break;
    }

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        const auto* duration = static_cast<const uint8_t*>(
            ezb_app_signal_get_params(app_signal));
        if (duration != nullptr) {
            ESP_LOGI(kTag, "Permit-join window: %u seconds", *duration);
        }
        break;
    }

    default:
        break;
    }

    return true;
}

void list_devices()
{
    if (!s_network_ready.load()) {
        std::printf("Zigbee network is not ready yet.\n");
        s_selection_count = 0;
        std::printf("Device list commands: Rename <number>, Remove <number>\n");
        return;
    }

    refresh_neighbors();

    std::array<DeviceRecord, kMaxChildren> snapshot{};
    std::array<int64_t, kMaxChildren> last_seen_snapshot{};
    std::array<int8_t, kMaxChildren> rssi_snapshot{};
    std::array<bool, kMaxChildren> rssi_valid_snapshot{};
    size_t count = 0;
    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < s_devices.size(); ++i) {
        if (s_devices[i].active) {
            snapshot[count] = s_devices[i];
            last_seen_snapshot[count] = s_last_seen_us[i];
            rssi_snapshot[count] = s_rssi[i];
            rssi_valid_snapshot[count] = s_rssi_valid[i];
            s_selection[count++] = s_devices[i].ieee_address;
        }
    }
    portEXIT_CRITICAL(&s_devices_mux);

    s_selection_count = count;
    if (count == 0) {
        std::printf("No joined devices are currently visible.\n");
        std::printf("Device list commands: Rename <number>, Remove <number>\n");
        return;
    }

    std::printf("Joined devices:\n");
    const int64_t now = esp_timer_get_time();
    for (size_t i = 0; i < count; ++i) {
        const char* name = snapshot[i].name[0] == '\0' ? "(unnamed)" : snapshot[i].name;
        char last_heard[32]{};
        if (last_seen_snapshot[i] == 0) {
            std::snprintf(last_heard, sizeof(last_heard), "N/A");
        } else {
            const uint64_t elapsed_seconds =
                static_cast<uint64_t>(now - last_seen_snapshot[i]) / 1000000;
            if (elapsed_seconds < 60) {
                std::snprintf(last_heard, sizeof(last_heard), "%llus ago",
                              static_cast<unsigned long long>(elapsed_seconds));
            } else if (elapsed_seconds < 60 * 60) {
                std::snprintf(last_heard, sizeof(last_heard), "%llum %llus ago",
                              static_cast<unsigned long long>(elapsed_seconds / 60),
                              static_cast<unsigned long long>(elapsed_seconds % 60));
            } else if (elapsed_seconds < 24 * 60 * 60) {
                std::snprintf(last_heard, sizeof(last_heard), "%lluh %llum ago",
                              static_cast<unsigned long long>(elapsed_seconds / 3600),
                              static_cast<unsigned long long>((elapsed_seconds % 3600) / 60));
            } else {
                std::snprintf(last_heard, sizeof(last_heard), "%llud %lluh ago",
                              static_cast<unsigned long long>(elapsed_seconds / 86400),
                              static_cast<unsigned long long>((elapsed_seconds % 86400) / 3600));
            }
        }
        if (rssi_valid_snapshot[i]) {
            std::printf("%u) %s - short 0x%04x, IEEE 0x%016llx, last heard: %s, RSSI: %d dBm\n",
                        static_cast<unsigned>(i + 1), name, snapshot[i].short_address,
                        static_cast<unsigned long long>(snapshot[i].ieee_address), last_heard,
                        static_cast<int>(rssi_snapshot[i]));
        } else {
            std::printf("%u) %s - short 0x%04x, IEEE 0x%016llx, last heard: %s, RSSI: N/A\n",
                        static_cast<unsigned>(i + 1), name, snapshot[i].short_address,
                        static_cast<unsigned long long>(snapshot[i].ieee_address), last_heard);
        }
    }
    std::printf("Device list commands: Rename <number>, Remove <number>\n");
}

bool parse_device_number(char* argument, size_t& selection_index)
{
    while (*argument == ' ' || *argument == '\t') {
        ++argument;
    }
    size_t length = std::strlen(argument);
    while (length > 0 && (argument[length - 1] == ' ' || argument[length - 1] == '\t')) {
        argument[--length] = '\0';
    }

    char* end = nullptr;
    const long choice = std::strtol(argument, &end, 10);
    if (length == 0 || end == argument || *end != '\0' || choice < 1 ||
        static_cast<size_t>(choice) > s_selection_count) {
        return false;
    }
    selection_index = static_cast<size_t>(choice - 1);
    return true;
}

void return_to_device_list()
{
    s_rename_stage = RenameStage::None;
    if (s_device_list_view) {
        list_devices();
    }
}

void handle_rename_input(char* input)
{
    if (s_rename_stage == RenameStage::ConfirmRemove) {
        if (strcasecmp(input, "YES") != 0) {
            std::printf("Remove cancelled.\n");
            return_to_device_list();
            return;
        }

        uint16_t short_address = EZB_NWK_ADDR_UNKNOWN;
        size_t name_slot = s_persisted_names.size();
        PersistedNameRecord previous_name{};
        portENTER_CRITICAL(&s_devices_mux);
        size_t device_slot = s_devices.size();
        for (size_t i = 0; i < s_devices.size(); ++i) {
            if (s_devices[i].active &&
                s_devices[i].ieee_address == s_selected_ieee_address) {
                device_slot = i;
                short_address = s_devices[i].short_address;
                break;
            }
        }
        if (device_slot < s_devices.size()) {
            for (size_t i = 0; i < s_persisted_names.size(); ++i) {
                if (s_persisted_names[i].ieee_address == s_selected_ieee_address) {
                    name_slot = i;
                    previous_name = s_persisted_names[i];
                    s_persisted_names[i] = PersistedNameRecord{};
                    break;
                }
            }
            s_devices[device_slot] = DeviceRecord{};
            s_sensor_values[device_slot] = SensorValues{};
            s_last_seen_us[device_slot] = 0;
            s_rssi[device_slot] = 0;
            s_rssi_valid[device_slot] = false;
        }
        portEXIT_CRITICAL(&s_devices_mux);

        if (device_slot == s_devices.size()) {
            std::printf("Device is no longer joined; removal cancelled.\n");
            return_to_device_list();
            return;
        }

        bool leave_request_queued = false;
        if (s_network_ready.load() && short_address != EZB_NWK_ADDR_UNKNOWN) {
            ezb_zdo_nwk_mgmt_leave_req_t request{};
            request.dst_nwk_addr = short_address;
            request.field.device_addr.u64 = s_selected_ieee_address;
            request.field.remove_children = false;
            request.field.rejoin = false;

            esp_zigbee_lock_acquire(portMAX_DELAY);
            const ezb_err_t result = ezb_zdo_nwk_mgmt_leave_req(&request);
            esp_zigbee_lock_release();
            leave_request_queued = result == EZB_ERR_NONE;
            if (!leave_request_queued) {
                std::printf("Could not request Zigbee leave (error 0x%04x).\n", result);
            }
        }

        if (name_slot < s_persisted_names.size() && !save_device_names()) {
            portENTER_CRITICAL(&s_devices_mux);
            s_persisted_names[name_slot] = previous_name;
            portEXIT_CRITICAL(&s_devices_mux);
            std::printf("Device was removed locally, but its saved name could not be cleared.\n");
        }
        if (leave_request_queued) {
            std::printf("Device removed locally; Zigbee leave request sent. The device may rejoin if it does not receive or honor the request.\n");
        } else {
            std::printf("Device removed from the local list. It may reappear if it remains on the Zigbee network.\n");
        }
        return_to_device_list();
        return;
    }

    if (s_rename_stage != RenameStage::EnterName) {
        return;
    }

    char* name = input;
    while (*name == ' ' || *name == '\t') {
        ++name;
    }
    size_t length = std::strlen(name);
    while (length > 0 && (name[length - 1] == ' ' || name[length - 1] == '\t')) {
        name[--length] = '\0';
    }
    if (length == 0) {
        std::printf("Rename cancelled.\n");
        return_to_device_list();
        return;
    }
    if (length >= kDeviceNameSize) {
        std::printf("Name is too long; rename cancelled.\n");
        return_to_device_list();
        return;
    }

    char previous_name[kDeviceNameSize]{};
    PersistedNameRecord previous_record{};
    size_t name_slot = s_persisted_names.size();
    bool added_name_record = false;
    portENTER_CRITICAL(&s_devices_mux);
    DeviceRecord* selected = nullptr;
    for (DeviceRecord& device : s_devices) {
        if (device.active && device.ieee_address == s_selected_ieee_address) {
            selected = &device;
            break;
        }
    }
    if (selected == nullptr) {
        portEXIT_CRITICAL(&s_devices_mux);
        std::printf("Device is no longer joined; rename cancelled.\n");
        return_to_device_list();
        return;
    }
    for (size_t i = 0; i < s_persisted_names.size(); ++i) {
        if (s_persisted_names[i].ieee_address == s_selected_ieee_address) {
            name_slot = i;
            break;
        }
        if (name_slot == s_persisted_names.size() &&
            s_persisted_names[i].ieee_address == 0) {
            name_slot = i;
        }
    }
    if (name_slot == s_persisted_names.size()) {
        portEXIT_CRITICAL(&s_devices_mux);
        std::printf("Saved-name storage is full; cannot rename this device.\n");
        return_to_device_list();
        return;
    }
    previous_record = s_persisted_names[name_slot];
    added_name_record = previous_record.ieee_address == 0;
    std::memcpy(previous_name, selected->name, sizeof(previous_name));
    std::snprintf(selected->name, sizeof(selected->name), "%s", name);
    s_persisted_names[name_slot].ieee_address = s_selected_ieee_address;
    std::snprintf(s_persisted_names[name_slot].name,
                  sizeof(s_persisted_names[name_slot].name), "%s", name);
    portEXIT_CRITICAL(&s_devices_mux);

    if (save_device_names()) {
        std::printf("Device renamed to \"%s\".\n", name);
    } else {
        portENTER_CRITICAL(&s_devices_mux);
        for (DeviceRecord& device : s_devices) {
            if (device.active && device.ieee_address == s_selected_ieee_address) {
                std::memcpy(device.name, previous_name, sizeof(previous_name));
                break;
            }
        }
        s_persisted_names[name_slot] = previous_record;
        if (added_name_record) {
            s_persisted_names[name_slot] = PersistedNameRecord{};
        }
        portEXIT_CRITICAL(&s_devices_mux);
    }
    return_to_device_list();
}

void handle_timeout_command(char* argument)
{
    while (*argument == ' ' || *argument == '\t') {
        ++argument;
    }
    size_t length = std::strlen(argument);
    while (length > 0 && (argument[length - 1] == ' ' || argument[length - 1] == '\t')) {
        argument[--length] = '\0';
    }

    if (length == 0) {
        std::printf("Sensor timeout is %u minute(s).\n",
                    static_cast<unsigned>(s_sensor_stale_after_minutes.load()));
        return;
    }

    char* end = nullptr;
    const long minutes = std::strtol(argument, &end, 10);
    if (end == argument || *end != '\0' || minutes < 1 ||
        minutes > kMaximumSensorStaleAfterMinutes) {
        std::printf("Usage: Timeout [minutes] (1-%u; default %u).\n",
                    static_cast<unsigned>(kMaximumSensorStaleAfterMinutes),
                    static_cast<unsigned>(kDefaultSensorStaleAfterMinutes));
        return;
    }

    const uint32_t new_timeout = static_cast<uint32_t>(minutes);
    if (!save_sensor_timeout(new_timeout)) {
        std::printf("Sensor timeout remains %u minute(s).\n",
                    static_cast<unsigned>(s_sensor_stale_after_minutes.load()));
        return;
    }
    s_sensor_stale_after_minutes.store(new_timeout);
    std::printf("Sensor timeout set to %u minute(s) and saved.\n",
                static_cast<unsigned>(new_timeout));
}

void handle_serial_command(char* command)
{
    if (s_rename_stage != RenameStage::None) {
        handle_rename_input(command);
        return;
    }

    if (strncasecmp(command, "Timeout", 7) == 0 &&
        (command[7] == '\0' || command[7] == ' ' || command[7] == '\t')) {
        handle_timeout_command(command + 7);
    } else if (strcasecmp(command, "Scan") == 0) {
        if (!s_network_ready.load()) {
            std::printf("Network is not ready yet; try Scan again shortly.\n");
            return;
        }

        esp_zigbee_lock_acquire(portMAX_DELAY);
        const ezb_err_t result = ezb_bdb_open_network(kJoinUntilClosed);
        esp_zigbee_lock_release();
        if (result == EZB_ERR_NONE) {
            std::printf("Network open for pairing until Stop is received.\n");
        } else {
            std::printf("Failed to open the network for pairing (error 0x%04x).\n", result);
        }
    } else if (strcasecmp(command, "Stop") == 0) {
        if (!s_network_ready.load()) {
            std::printf("Network is not ready yet.\n");
            return;
        }

        esp_zigbee_lock_acquire(portMAX_DELAY);
        const ezb_err_t result = ezb_bdb_close_network();
        esp_zigbee_lock_release();
        if (result == EZB_ERR_NONE) {
            std::printf("Network closed to new pairings.\n");
        } else {
            std::printf("Failed to close the network (error 0x%04x).\n", result);
        }
    } else if (strcasecmp(command, "Devices") == 0) {
        s_device_list_view = true;
        list_devices();
    } else if (strncasecmp(command, "Rename", 6) == 0 &&
               (command[6] == '\0' || command[6] == ' ' || command[6] == '\t')) {
        if (!s_device_list_view) {
            std::printf("Run Devices first to view the device list.\n");
            return;
        }
        size_t selection_index = 0;
        if (!parse_device_number(command + 6, selection_index)) {
            std::printf("Usage: Rename <device number>.\n");
            return;
        }
        s_selected_ieee_address = s_selection[selection_index];
        s_rename_stage = RenameStage::EnterName;
        std::printf("Enter a name (up to %u characters), or leave blank to cancel:\n",
                    static_cast<unsigned>(kDeviceNameSize - 1));
    } else if (strncasecmp(command, "Remove", 6) == 0 &&
               (command[6] == '\0' || command[6] == ' ' || command[6] == '\t')) {
        if (!s_device_list_view) {
            std::printf("Run Devices first to view the device list.\n");
            return;
        }
        size_t selection_index = 0;
        if (!parse_device_number(command + 6, selection_index)) {
            std::printf("Usage: Remove <device number>.\n");
            return;
        }
        s_selected_ieee_address = s_selection[selection_index];
        s_rename_stage = RenameStage::ConfirmRemove;
        std::printf("Remove device %u from the local list and request Zigbee leave? Type YES to confirm, or anything else to cancel:\n",
                    static_cast<unsigned>(selection_index + 1));
    } else if (command[0] != '\0') {
        std::printf("Unknown command. Use Scan, Stop, Devices, or Timeout [minutes].\n");
    }
}

void serial_command_task(void*)
{
    char line[64]{};
    size_t length = 0;
    bool discard_line = false;
    bool previous_was_carriage_return = false;
    std::printf("Serial commands: Scan, Stop, Devices, Timeout [minutes]\n");

    while (true) {
        const int input = std::getchar();
        if (input == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        const char character = static_cast<char>(input);
        if (character == '\r' || character == '\n') {
            if (character == '\n' && previous_was_carriage_return) {
                previous_was_carriage_return = false;
                continue;
            }
            previous_was_carriage_return = character == '\r';

            if (!discard_line) {
                line[length] = '\0';
                handle_serial_command(line);
            } else {
                std::printf("Serial command too long; command discarded.\n");
            }
            length = 0;
            discard_line = false;
            continue;
        }

        previous_was_carriage_return = false;
        if (character == '\b' || character == 0x7f) {
            if (!discard_line && length > 0) {
                --length;
            }
            continue;
        }

        if (discard_line) {
            continue;
        }
        if (length + 1 < sizeof(line)) {
            line[length++] = character;
        } else {
            length = 0;
            discard_line = true;
        }
    }
}

void zigbee_task(void*)
{
    esp_zigbee_config_t config{};
    config.device_config.install_code_policy = false;
    config.device_config.device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR;
    config.device_config.zczr_config.max_children = kMaxChildren;
    config.platform_config.storage_partition_name = "nvs";
    config.platform_config.radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE;

    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(zigbee_signal_handler));
    ezb_zcl_raw_command_handler_register(zigbee_raw_frame_handler);

    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();

    vTaskDelete(nullptr);
}

}  // namespace

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    select_external_antenna();
    load_device_names();
    load_sensor_timeout();

    if (xTaskCreate(zigbee_task, "zigbee", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create Zigbee task");
        return;
    }

    if (xTaskCreate(device_refresh_task, "device_refresh", 3072, nullptr, 3, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create device refresh task");
    }

    if (xTaskCreate(epaper_display_task, "epaper_display", 8192, nullptr, 2, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create e-paper display task");
    }

    if (xTaskCreate(serial_command_task, "serial_commands", 3072, nullptr, 4, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create serial command task");
    }
}
