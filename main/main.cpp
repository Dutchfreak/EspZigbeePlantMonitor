#include <atomic>
#include <array>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <strings.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {

constexpr char kTag[] = "zigbee_host";
constexpr uint8_t kJoinUntilClosed = 0xFF;
constexpr uint8_t kMaxChildren = 10;
constexpr size_t kMaxKnownDevices = 16;
constexpr size_t kDeviceNameSize = 32;
constexpr uint16_t kTuyaClusterId = 0xEF00;
constexpr uint8_t kTuyaDataReportCommand = 0x02;
constexpr char kNameNamespace[] = "zb_names";
constexpr char kNameRecordsKey[] = "records";

struct DeviceRecord {
    uint64_t ieee_address = 0;
    uint16_t short_address = EZB_NWK_ADDR_UNKNOWN;
    bool active = false;
    char name[kDeviceNameSize]{};
};

using PersistedNames = std::array<DeviceRecord, kMaxKnownDevices>;

std::atomic<bool> s_network_ready{false};
portMUX_TYPE s_devices_mux = portMUX_INITIALIZER_UNLOCKED;
std::array<DeviceRecord, kMaxKnownDevices> s_devices{};

enum class RenameStage {
    None,
    ChooseDevice,
    EnterName,
};

RenameStage s_rename_stage = RenameStage::None;
std::array<size_t, kMaxKnownDevices> s_selection{};
size_t s_selection_count = 0;
size_t s_selected_device = 0;

void load_device_names()
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(kNameNamespace, NVS_READONLY, &handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    ESP_ERROR_CHECK(result);

    PersistedNames names{};
    size_t size = sizeof(names);
    result = nvs_get_blob(handle, kNameRecordsKey, names.data(), &size);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    ESP_ERROR_CHECK(result);
    ESP_ERROR_CHECK(size == sizeof(names) ? ESP_OK : ESP_ERR_INVALID_SIZE);

    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < names.size(); ++i) {
        if (names[i].ieee_address != 0) {
            s_devices[i].ieee_address = names[i].ieee_address;
            std::memcpy(s_devices[i].name, names[i].name, kDeviceNameSize);
            s_devices[i].name[kDeviceNameSize - 1] = '\0';
        }
    }
    portEXIT_CRITICAL(&s_devices_mux);
}

bool save_device_names()
{
    PersistedNames names{};
    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < s_devices.size(); ++i) {
        names[i].ieee_address = s_devices[i].ieee_address;
        std::memcpy(names[i].name, s_devices[i].name, kDeviceNameSize);
    }
    portEXIT_CRITICAL(&s_devices_mux);

    nvs_handle_t handle;
    esp_err_t result = nvs_open(kNameNamespace, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "Could not open device-name storage: %s", esp_err_to_name(result));
        return false;
    }
    result = nvs_set_blob(handle, kNameRecordsKey, names.data(), sizeof(names));
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "Could not save device names: %s", esp_err_to_name(result));
        return false;
    }
    return true;
}

size_t find_or_add_device(uint64_t ieee_address, uint16_t short_address, bool active)
{
    size_t free_slot = s_devices.size();
    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < s_devices.size(); ++i) {
        if (s_devices[i].ieee_address == ieee_address) {
            s_devices[i].short_address = short_address;
            s_devices[i].active = active;
            portEXIT_CRITICAL(&s_devices_mux);
            return i;
        }
        if (free_slot == s_devices.size() && s_devices[i].ieee_address == 0) {
            free_slot = i;
        }
    }
    if (free_slot < s_devices.size()) {
        s_devices[free_slot].ieee_address = ieee_address;
        s_devices[free_slot].short_address = short_address;
        s_devices[free_slot].active = active;
    }
    portEXIT_CRITICAL(&s_devices_mux);
    return free_slot;
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
        find_or_add_device(neighbor.ieee_addr.u64, neighbor.short_addr, true);
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
            for (DeviceRecord& record : s_devices) {
                if (record.ieee_address == device->device_addr.u64) {
                    record.active = false;
                    record.short_address = EZB_NWK_ADDR_UNKNOWN;
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

void start_device_rename()
{
    if (!s_network_ready.load()) {
        ESP_LOGW(kTag, "Zigbee network is not ready yet");
        return;
    }

    refresh_neighbors();

    std::array<DeviceRecord, kMaxKnownDevices> snapshot{};
    size_t count = 0;
    portENTER_CRITICAL(&s_devices_mux);
    for (size_t i = 0; i < s_devices.size(); ++i) {
        if (s_devices[i].active) {
            snapshot[count] = s_devices[i];
            s_selection[count++] = i;
        }
    }
    portEXIT_CRITICAL(&s_devices_mux);

    if (count == 0) {
        ESP_LOGI(kTag, "No joined devices are currently visible");
        return;
    }

    s_selection_count = count;
    ESP_LOGI(kTag, "Joined devices:");
    for (size_t i = 0; i < count; ++i) {
        const char* name = snapshot[i].name[0] == '\0' ? "(unnamed)" : snapshot[i].name;
        ESP_LOGI(kTag, "%u) %s - short 0x%04x, IEEE 0x%016llx",
                 static_cast<unsigned>(i + 1), name, snapshot[i].short_address,
                 static_cast<unsigned long long>(snapshot[i].ieee_address));
    }
    ESP_LOGI(kTag, "Enter a device number to rename, or 0 to cancel:");
    s_rename_stage = RenameStage::ChooseDevice;
}

void handle_rename_input(char* input)
{
    if (s_rename_stage == RenameStage::ChooseDevice) {
        char* end = nullptr;
        const long choice = std::strtol(input, &end, 10);
        if (end == input || *end != '\0' || choice < 0 ||
            static_cast<size_t>(choice) > s_selection_count) {
            ESP_LOGW(kTag, "Invalid selection; rename cancelled");
            s_rename_stage = RenameStage::None;
            return;
        }
        if (choice == 0) {
            ESP_LOGI(kTag, "Rename cancelled");
            s_rename_stage = RenameStage::None;
            return;
        }
        s_selected_device = s_selection[static_cast<size_t>(choice - 1)];
        s_rename_stage = RenameStage::EnterName;
        ESP_LOGI(kTag, "Enter a name (up to %u characters), or leave blank to cancel:",
                 static_cast<unsigned>(kDeviceNameSize - 1));
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
        ESP_LOGI(kTag, "Rename cancelled");
        s_rename_stage = RenameStage::None;
        return;
    }
    if (length >= kDeviceNameSize) {
        ESP_LOGW(kTag, "Name is too long; rename cancelled");
        s_rename_stage = RenameStage::None;
        return;
    }

    char previous_name[kDeviceNameSize]{};
    portENTER_CRITICAL(&s_devices_mux);
    DeviceRecord& selected = s_devices[s_selected_device];
    if (!selected.active) {
        portEXIT_CRITICAL(&s_devices_mux);
        ESP_LOGW(kTag, "Device is no longer joined; rename cancelled");
        s_rename_stage = RenameStage::None;
        return;
    }
    std::memcpy(previous_name, selected.name, sizeof(previous_name));
    std::snprintf(selected.name, sizeof(selected.name), "%s", name);
    portEXIT_CRITICAL(&s_devices_mux);

    if (save_device_names()) {
        ESP_LOGI(kTag, "Device renamed to \"%s\"", name);
    } else {
        portENTER_CRITICAL(&s_devices_mux);
        std::memcpy(s_devices[s_selected_device].name, previous_name, sizeof(previous_name));
        portEXIT_CRITICAL(&s_devices_mux);
    }
    s_rename_stage = RenameStage::None;
}

void handle_serial_command(char* command)
{
    if (s_rename_stage != RenameStage::None) {
        handle_rename_input(command);
        return;
    }

    if (strcasecmp(command, "Scan") == 0) {
        if (!s_network_ready.load()) {
            ESP_LOGW(kTag, "Network is not ready yet; try Scan again shortly");
            return;
        }

        esp_zigbee_lock_acquire(portMAX_DELAY);
        const ezb_err_t result = ezb_bdb_open_network(kJoinUntilClosed);
        esp_zigbee_lock_release();
        if (result == EZB_ERR_NONE) {
            ESP_LOGI(kTag, "Network open for pairing until Stop is received");
        } else {
            ESP_LOGE(kTag, "Failed to open the network for pairing (error 0x%04x)", result);
        }
    } else if (strcasecmp(command, "Stop") == 0) {
        if (!s_network_ready.load()) {
            ESP_LOGW(kTag, "Network is not ready yet");
            return;
        }

        esp_zigbee_lock_acquire(portMAX_DELAY);
        const ezb_err_t result = ezb_bdb_close_network();
        esp_zigbee_lock_release();
        if (result == EZB_ERR_NONE) {
            ESP_LOGI(kTag, "Network closed to new pairings");
        } else {
            ESP_LOGE(kTag, "Failed to close the network (error 0x%04x)", result);
        }
    } else if (strcasecmp(command, "Devices") == 0 || strcasecmp(command, "Rename") == 0) {
        start_device_rename();
    } else if (command[0] != '\0') {
        ESP_LOGW(kTag, "Unknown command. Use Scan, Stop, or Devices");
    }
}

void serial_command_task(void*)
{
    char line[64]{};
    size_t length = 0;
    bool discard_line = false;
    bool previous_was_carriage_return = false;
    ESP_LOGI(kTag, "Serial commands: Scan, Stop, Devices (list and rename)");

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
                ESP_LOGW(kTag, "Serial command too long; command discarded");
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
    load_device_names();

    if (xTaskCreate(zigbee_task, "zigbee", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create Zigbee task");
        return;
    }

    if (xTaskCreate(device_refresh_task, "device_refresh", 3072, nullptr, 3, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create device refresh task");
    }

    if (xTaskCreate(serial_command_task, "serial_commands", 3072, nullptr, 4, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create serial command task");
    }
}
