/**
 * Flic 2 ESP32 Platform Implementation
 */

#include "flic2_esp32_platform.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_defs.h"

static const char* TAG = "flic2_platform";

// NVS namespace for Flic2 storage
#define FLIC2_NVS_NAMESPACE "flic2"

// ============================================================================
// Time Functions
// ============================================================================

double flic2_platform_get_steady_time(void) {
    // esp_timer_get_time returns microseconds since boot
    int64_t us = esp_timer_get_time();
    return (double)us / 1000000.0;
}

double flic2_platform_get_utc_time(void) {
    struct timeval tv;
    if (gettimeofday(&tv, NULL) == 0) {
        return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
    }
    // Return 0 if system time not available
    return 0.0;
}

// ============================================================================
// Random Number Generation
// ============================================================================

void flic2_platform_get_random(uint8_t* buf, size_t len) {
    esp_fill_random(buf, len);
}

// ============================================================================
// NVS Storage Functions
// ============================================================================

static char* bd_addr_to_key(const uint8_t bd_addr[6], char* key_buf) {
    // Create key from BD address: "f_AABBCCDDEEFF"
    snprintf(key_buf, 16, "f_%02X%02X%02X%02X%02X%02X",
             bd_addr[5], bd_addr[4], bd_addr[3],
             bd_addr[2], bd_addr[1], bd_addr[0]);
    return key_buf;
}

esp_err_t flic2_storage_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase");
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    return err;
}

esp_err_t flic2_storage_save(const uint8_t bd_addr[6], const struct Flic2DbData* data) {
    nvs_handle_t handle;
    char key[16];

    esp_err_t err = nvs_open(FLIC2_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    bd_addr_to_key(bd_addr, key);
    err = nvs_set_blob(handle, key, data, sizeof(struct Flic2DbData));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved pairing for %02X:%02X:%02X:%02X:%02X:%02X",
                 bd_addr[5], bd_addr[4], bd_addr[3],
                 bd_addr[2], bd_addr[1], bd_addr[0]);
    }

    return err;
}

esp_err_t flic2_storage_load(const uint8_t bd_addr[6], struct Flic2DbData* data) {
    nvs_handle_t handle;
    char key[16];
    size_t size = sizeof(struct Flic2DbData);

    esp_err_t err = nvs_open(FLIC2_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    bd_addr_to_key(bd_addr, key);
    err = nvs_get_blob(handle, key, data, &size);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded pairing for %02X:%02X:%02X:%02X:%02X:%02X",
                 bd_addr[5], bd_addr[4], bd_addr[3],
                 bd_addr[2], bd_addr[1], bd_addr[0]);
    }

    return err;
}

esp_err_t flic2_storage_delete(const uint8_t bd_addr[6]) {
    nvs_handle_t handle;
    char key[16];

    esp_err_t err = nvs_open(FLIC2_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    bd_addr_to_key(bd_addr, key);
    err = nvs_erase_key(handle, key);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Deleted pairing for %02X:%02X:%02X:%02X:%02X:%02X",
                 bd_addr[5], bd_addr[4], bd_addr[3],
                 bd_addr[2], bd_addr[1], bd_addr[0]);
    }

    return err;
}

int flic2_storage_list_paired(uint8_t (*bd_addrs)[6], int max_count) {
    nvs_handle_t handle;
    nvs_iterator_t it = NULL;
    int count = 0;

    esp_err_t err = nvs_open(FLIC2_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return 0;
    }

    err = nvs_entry_find("nvs", FLIC2_NVS_NAMESPACE, NVS_TYPE_BLOB, &it);
    while (err == ESP_OK && count < max_count) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        // Parse key "f_AABBCCDDEEFF" back to BD address
        if (info.key[0] == 'f' && info.key[1] == '_' && strlen(info.key) == 14) {
            unsigned int addr[6];
            if (sscanf(info.key + 2, "%02X%02X%02X%02X%02X%02X",
                       &addr[0], &addr[1], &addr[2],
                       &addr[3], &addr[4], &addr[5]) == 6) {
                // Store in little-endian format as expected by flic2lib
                bd_addrs[count][0] = (uint8_t)addr[5];
                bd_addrs[count][1] = (uint8_t)addr[4];
                bd_addrs[count][2] = (uint8_t)addr[3];
                bd_addrs[count][3] = (uint8_t)addr[2];
                bd_addrs[count][4] = (uint8_t)addr[1];
                bd_addrs[count][5] = (uint8_t)addr[0];
                count++;
            }
        }

        err = nvs_entry_next(&it);
    }

    if (it != NULL) {
        nvs_release_iterator(it);
    }
    nvs_close(handle);

    return count;
}

// ============================================================================
// BLE Functions (Stub Implementations)
// TODO: Full implementation requires integration with flic2_manager
// ============================================================================

esp_err_t flic2_ble_init(void) {
    ESP_LOGI(TAG, "BLE init");

    // Release classic BT memory (we only use BLE)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BLE initialized successfully");
    return ESP_OK;
}

esp_err_t flic2_ble_deinit(void) {
    ESP_LOGI(TAG, "BLE deinit");

    esp_err_t err = esp_bluedroid_disable();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Bluedroid disable failed: %s", esp_err_to_name(err));
    }

    err = esp_bluedroid_deinit();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Bluedroid deinit failed: %s", esp_err_to_name(err));
    }

    err = esp_bt_controller_disable();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BT controller disable failed: %s", esp_err_to_name(err));
    }

    err = esp_bt_controller_deinit();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BT controller deinit failed: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

esp_err_t flic2_ble_start_scan(uint32_t duration_seconds) {
    ESP_LOGI(TAG, "Starting BLE scan for %lu seconds", (unsigned long)duration_seconds);

    static esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,  // 50ms
        .scan_window = 0x30,    // 30ms
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };

    esp_err_t err = esp_ble_gap_set_scan_params(&scan_params);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set scan params failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ble_gap_start_scanning(duration_seconds);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Start scanning failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t flic2_ble_stop_scan(void) {
    ESP_LOGI(TAG, "Stopping BLE scan");
    return esp_ble_gap_stop_scanning();
}

esp_err_t flic2_ble_connect(const uint8_t bd_addr[6]) {
    ESP_LOGI(TAG, "Connecting to %02X:%02X:%02X:%02X:%02X:%02X",
             bd_addr[5], bd_addr[4], bd_addr[3],
             bd_addr[2], bd_addr[1], bd_addr[0]);

    // Open GATT connection - actual connection handling is done in flic2_manager
    // through the GATTC event callbacks
    esp_err_t err = esp_ble_gattc_open(
        0,  // gattc_if - will be set properly by manager
        (uint8_t*)bd_addr,
        BLE_ADDR_TYPE_PUBLIC,
        true  // direct connection
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GATTC open failed: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t flic2_ble_disconnect(const uint8_t bd_addr[6]) {
    ESP_LOGI(TAG, "Disconnecting from %02X:%02X:%02X:%02X:%02X:%02X",
             bd_addr[5], bd_addr[4], bd_addr[3],
             bd_addr[2], bd_addr[1], bd_addr[0]);

    // Note: Disconnect is handled in flic2_manager using the conn_id
    // This is a placeholder - actual disconnect uses esp_ble_gattc_close(gattc_if, conn_id)
    ESP_LOGW(TAG, "Disconnect requires conn_id from manager");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t flic2_ble_write(flic2_button_instance_t* instance, const uint8_t* data, size_t len) {
    if (instance == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (instance->conn_state != FLIC2_CONN_STATE_CONNECTED &&
        instance->conn_state != FLIC2_CONN_STATE_SESSION_ACTIVE) {
        ESP_LOGW(TAG, "Cannot write - not connected");
        return ESP_ERR_INVALID_STATE;
    }

    if (instance->write_handle == 0) {
        ESP_LOGE(TAG, "Write handle not discovered");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Writing %d bytes to handle 0x%04X", (int)len, instance->write_handle);

    // Use write without response for Flic2 protocol
    esp_err_t err = esp_ble_gattc_write_char(
        instance->gattc_if,
        instance->conn_id,
        instance->write_handle,
        len,
        (uint8_t*)data,
        ESP_GATT_WRITE_TYPE_NO_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GATTC write failed: %s", esp_err_to_name(err));
    }

    return err;
}
