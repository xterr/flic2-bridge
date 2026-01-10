/**
 * Flic 2 Manager Implementation
 */

#include "flic2_manager.h"
#include "flic2.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"

static const char* TAG = "flic2_manager";

// GATT client app ID
#define FLIC2_GATTC_APP_ID 0

// Target MTU for Flic2
#define FLIC2_TARGET_MTU 140

// Flic2 service and characteristic UUIDs
static const uint8_t FLIC2_SERVICE_UUID[] = FLIC2_SERVICE_UUID_128;
static const uint8_t FLIC2_CHAR_WRITE_UUID[] = FLIC2_CHAR_WRITE_UUID_128;
static const uint8_t FLIC2_CHAR_NOTIFY_UUID[] = FLIC2_CHAR_NOTIFY_UUID_128;

// Manager state
static struct {
    bool initialized;
    flic2_callbacks_t callbacks;
    flic2_button_instance_t buttons[FLIC2_MAX_BUTTONS];
    int button_count;
    esp_gatt_if_t gattc_if;
    bool scanning;
    SemaphoreHandle_t mutex;
    uint64_t rand_nonce;
} manager = {0};

// Forward declarations
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param);
static flic2_button_instance_t* find_button_by_addr(const uint8_t bd_addr[6]);
static flic2_button_instance_t* find_button_by_conn_id(uint16_t conn_id);
static flic2_button_instance_t* find_button_by_notify_handle(uint16_t handle);
static flic2_button_instance_t* allocate_button(const uint8_t bd_addr[6]);
static void process_flic2_events(flic2_button_instance_t* instance);
static void handle_db_update(flic2_button_instance_t* instance, struct Flic2DbUpdate* db_update);

// ============================================================================
// Initialization
// ============================================================================

esp_err_t flic2_manager_init(const flic2_callbacks_t* callbacks) {
    ESP_LOGW(TAG, "=== flic2_manager_init() called ===");

    if (manager.initialized) {
        ESP_LOGW(TAG, "Already initialized, returning");
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize storage
    ESP_LOGI(TAG, "Step 1: Init storage...");
    esp_err_t err = flic2_storage_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init storage: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Storage OK");

    // Create mutex
    ESP_LOGI(TAG, "Step 2: Create mutex...");
    manager.mutex = xSemaphoreCreateMutex();
    if (manager.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Mutex OK");

    // Store callbacks
    if (callbacks) {
        memcpy(&manager.callbacks, callbacks, sizeof(flic2_callbacks_t));
    }

    // Initialize Bluetooth controller (may already be initialized by ESPHome)
    ESP_LOGI(TAG, "Step 3: BT controller init...");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "BT controller already initialized");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(err));
        return err;
    } else {
        ESP_LOGI(TAG, "BT controller init OK");
    }

    ESP_LOGI(TAG, "Step 4: BT controller enable...");
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "BT controller already enabled");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(err));
        return err;
    } else {
        ESP_LOGI(TAG, "BT controller enable OK");
    }

    // Initialize Bluedroid (may already be initialized by ESPHome)
    ESP_LOGI(TAG, "Step 5: Bluedroid init...");
    err = esp_bluedroid_init();
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Bluedroid already initialized");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(err));
        return err;
    } else {
        ESP_LOGI(TAG, "Bluedroid init OK");
    }

    ESP_LOGI(TAG, "Step 6: Bluedroid enable...");
    err = esp_bluedroid_enable();
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Bluedroid already enabled");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(err));
        return err;
    } else {
        ESP_LOGI(TAG, "Bluedroid enable OK");
    }

    // Register GAP callback (this may fail if ESPHome already registered one)
    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GAP register callback returned: %s (may be OK if ESPHome registered one)", esp_err_to_name(err));
        // Don't return - try to continue anyway
    } else {
        ESP_LOGI(TAG, "GAP callback registered");
    }

    // Register GATTC callback
    err = esp_ble_gattc_register_callback(gattc_event_handler);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GATTC register callback returned: %s", esp_err_to_name(err));
        // Don't return - try to continue anyway
    } else {
        ESP_LOGI(TAG, "GATTC callback registered");
    }

    // Register GATTC app
    err = esp_ble_gattc_app_register(FLIC2_GATTC_APP_ID);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GATTC app register failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "GATTC app registered");

    // Set MTU
    err = esp_ble_gatt_set_local_mtu(FLIC2_TARGET_MTU);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Set MTU failed: %s (non-fatal)", esp_err_to_name(err));
    }

    // Load paired buttons
    uint8_t paired_addrs[FLIC2_MAX_BUTTONS][6];
    int paired_count = flic2_storage_list_paired(paired_addrs, FLIC2_MAX_BUTTONS);

    for (int i = 0; i < paired_count; i++) {
        flic2_button_instance_t* instance = allocate_button(paired_addrs[i]);
        if (instance) {
            struct Flic2DbData db_data;
            if (flic2_storage_load(paired_addrs[i], &db_data) == ESP_OK) {
                uint8_t rand_seed[16];
                flic2_platform_get_random(rand_seed, sizeof(rand_seed));
                flic2_init(&instance->button, paired_addrs[i], &db_data,
                           rand_seed, manager.rand_nonce++);
                instance->paired = true;

                ESP_LOGI(TAG, "Loaded paired button: %s",
                         db_data.name.len > 0 ? db_data.name.value : "(unnamed)");
            }
        }
    }

    manager.initialized = true;
    ESP_LOGI(TAG, "Flic2 Manager initialized with %d paired buttons", paired_count);

    return ESP_OK;
}

esp_err_t flic2_manager_deinit(void) {
    if (!manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Disconnect all buttons
    for (int i = 0; i < manager.button_count; i++) {
        if (manager.buttons[i].conn_state != FLIC2_CONN_STATE_DISCONNECTED) {
            flic2_manager_disconnect(manager.buttons[i].bd_addr);
        }
    }

    esp_ble_gattc_app_unregister(manager.gattc_if);
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    vSemaphoreDelete(manager.mutex);

    memset(&manager, 0, sizeof(manager));
    ESP_LOGI(TAG, "Flic2 Manager deinitialized");

    return ESP_OK;
}

// ============================================================================
// Scanning
// ============================================================================

esp_err_t flic2_manager_start_scan(uint32_t duration_seconds) {
    if (!manager.initialized) {
        ESP_LOGE(TAG, "Cannot start scan - manager not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting BLE scan for Flic2 buttons (%lu sec)", duration_seconds);

    esp_ble_scan_params_t scan_params = {
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

    // Scanning will start after ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT
    manager.scanning = true;

    return ESP_OK;
}

esp_err_t flic2_manager_stop_scan(void) {
    if (!manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    manager.scanning = false;
    return esp_ble_gap_stop_scanning();
}

// ============================================================================
// Connection Management
// ============================================================================

esp_err_t flic2_manager_pair(const uint8_t bd_addr[6]) {
    if (!manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Pairing with %02X:%02X:%02X:%02X:%02X:%02X",
             bd_addr[5], bd_addr[4], bd_addr[3],
             bd_addr[2], bd_addr[1], bd_addr[0]);

    flic2_button_instance_t* instance = find_button_by_addr(bd_addr);
    if (!instance) {
        instance = allocate_button(bd_addr);
        if (!instance) {
            ESP_LOGE(TAG, "No slot available for new button");
            return ESP_ERR_NO_MEM;
        }
    }

    // Initialize for new pairing (no stored data)
    uint8_t rand_seed[16];
    flic2_platform_get_random(rand_seed, sizeof(rand_seed));
    flic2_init(&instance->button, bd_addr, NULL, rand_seed, manager.rand_nonce++);
    instance->paired = false;
    instance->conn_state = FLIC2_CONN_STATE_CONNECTING;

    // Connect
    esp_ble_addr_type_t addr_type = BLE_ADDR_TYPE_PUBLIC;
    return esp_ble_gattc_open(manager.gattc_if, (uint8_t*)bd_addr, addr_type, true);
}

esp_err_t flic2_manager_connect(const uint8_t bd_addr[6]) {
    if (!manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    flic2_button_instance_t* instance = find_button_by_addr(bd_addr);
    if (!instance || !instance->paired) {
        ESP_LOGE(TAG, "Button not found or not paired");
        return ESP_ERR_NOT_FOUND;
    }

    if (instance->conn_state != FLIC2_CONN_STATE_DISCONNECTED) {
        ESP_LOGW(TAG, "Button already connecting/connected");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Connecting to %02X:%02X:%02X:%02X:%02X:%02X",
             bd_addr[5], bd_addr[4], bd_addr[3],
             bd_addr[2], bd_addr[1], bd_addr[0]);

    instance->conn_state = FLIC2_CONN_STATE_CONNECTING;

    esp_ble_addr_type_t addr_type = BLE_ADDR_TYPE_PUBLIC;
    return esp_ble_gattc_open(manager.gattc_if, (uint8_t*)bd_addr, addr_type, true);
}

esp_err_t flic2_manager_disconnect(const uint8_t bd_addr[6]) {
    flic2_button_instance_t* instance = find_button_by_addr(bd_addr);
    if (!instance) {
        return ESP_ERR_NOT_FOUND;
    }

    if (instance->conn_state == FLIC2_CONN_STATE_DISCONNECTED) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Disconnecting from %02X:%02X:%02X:%02X:%02X:%02X",
             bd_addr[5], bd_addr[4], bd_addr[3],
             bd_addr[2], bd_addr[1], bd_addr[0]);

    flic2_on_disconnected(&instance->button);
    instance->conn_state = FLIC2_CONN_STATE_DISCONNECTED;

    return esp_ble_gattc_close(manager.gattc_if, instance->conn_id);
}

esp_err_t flic2_manager_unpair(const uint8_t bd_addr[6]) {
    flic2_button_instance_t* instance = find_button_by_addr(bd_addr);
    if (!instance) {
        return ESP_ERR_NOT_FOUND;
    }

    // Disconnect first
    if (instance->conn_state != FLIC2_CONN_STATE_DISCONNECTED) {
        flic2_manager_disconnect(bd_addr);
    }

    // Remove from storage
    flic2_storage_delete(bd_addr);

    // Mark as unpaired
    instance->paired = false;

    // Notify callback
    if (manager.callbacks.on_unpaired) {
        manager.callbacks.on_unpaired((uint8_t*)bd_addr);
    }

    return ESP_OK;
}

// ============================================================================
// Query Functions
// ============================================================================

int flic2_manager_get_paired(uint8_t (*bd_addrs)[6], int max_count) {
    return flic2_storage_list_paired(bd_addrs, max_count);
}

flic2_button_instance_t* flic2_manager_get_button(const uint8_t bd_addr[6]) {
    return find_button_by_addr(bd_addr);
}

int flic2_manager_get_connected_count(void) {
    int count = 0;
    for (int i = 0; i < manager.button_count; i++) {
        if (manager.buttons[i].conn_state == FLIC2_CONN_STATE_SESSION_ACTIVE) {
            count++;
        }
    }
    return count;
}

bool flic2_manager_is_connected(const uint8_t bd_addr[6]) {
    flic2_button_instance_t* instance = find_button_by_addr(bd_addr);
    return instance && instance->conn_state == FLIC2_CONN_STATE_SESSION_ACTIVE;
}

bool flic2_manager_get_name(const uint8_t bd_addr[6], char* name_buf) {
    flic2_button_instance_t* instance = find_button_by_addr(bd_addr);
    if (!instance || instance->button.d.name.len == 0) {
        return false;
    }
    strncpy(name_buf, instance->button.d.name.value, 24);
    return true;
}

uint16_t flic2_manager_get_battery_mv(const uint8_t bd_addr[6]) {
    flic2_button_instance_t* instance = find_button_by_addr(bd_addr);
    if (!instance) {
        return 0;
    }
    return instance->button.d.battery_voltage_millivolt;
}

// ============================================================================
// Main Loop
// ============================================================================

void flic2_manager_loop(void) {
    if (!manager.initialized) {
        return;
    }

    double current_time = flic2_platform_get_steady_time();
    int64_t current_us = esp_timer_get_time();

    xSemaphoreTake(manager.mutex, portMAX_DELAY);

    for (int i = 0; i < manager.button_count; i++) {
        flic2_button_instance_t* instance = &manager.buttons[i];

        // Check timer
        if (instance->timer_active && current_us >= instance->timer_target_us) {
            instance->timer_active = false;
            flic2_on_timer(&instance->button, current_time);
            process_flic2_events(instance);
        }
    }

    xSemaphoreGive(manager.mutex);
}

// ============================================================================
// GAP Event Handler
// ============================================================================

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                if (manager.scanning) {
                    esp_ble_gap_start_scanning(0);  // Scan indefinitely
                }
            }
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // Check for Flic2 service UUID in advertisement
                uint8_t* adv_data = param->scan_rst.ble_adv;
                uint8_t adv_len = param->scan_rst.adv_data_len;

                // Parse advertisement data looking for 128-bit service UUID
                int pos = 0;
                while (pos < adv_len) {
                    uint8_t len = adv_data[pos];
                    if (len == 0) break;

                    uint8_t type = adv_data[pos + 1];

                    // Complete 128-bit service UUID
                    if (type == 0x07 && len >= 17) {
                        if (memcmp(&adv_data[pos + 2], FLIC2_SERVICE_UUID, 16) == 0) {
                            ESP_LOGI(TAG, "Found Flic2 button: %02X:%02X:%02X:%02X:%02X:%02X RSSI: %d",
                                     param->scan_rst.bda[0], param->scan_rst.bda[1],
                                     param->scan_rst.bda[2], param->scan_rst.bda[3],
                                     param->scan_rst.bda[4], param->scan_rst.bda[5],
                                     param->scan_rst.rssi);
                        }
                    }

                    pos += len + 1;
                }
            }
            break;

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan stopped");
            manager.scanning = false;
            break;

        default:
            break;
    }
}

// ============================================================================
// GATTC Event Handler
// ============================================================================

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                 esp_ble_gattc_cb_param_t* param) {
    flic2_button_instance_t* instance = NULL;

    switch (event) {
        case ESP_GATTC_REG_EVT:
            if (param->reg.status == ESP_GATT_OK) {
                manager.gattc_if = gattc_if;
                ESP_LOGI(TAG, "GATTC registered, if=%d", gattc_if);
            } else {
                ESP_LOGE(TAG, "GATTC register failed: %d", param->reg.status);
            }
            break;

        case ESP_GATTC_OPEN_EVT:
            if (param->open.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Connected, conn_id=%d", param->open.conn_id);

                instance = find_button_by_addr(param->open.remote_bda);
                if (instance) {
                    instance->conn_id = param->open.conn_id;
                    instance->gattc_if = gattc_if;
                    instance->conn_state = FLIC2_CONN_STATE_DISCOVERING_SERVICES;

                    // Request MTU exchange
                    esp_ble_gattc_send_mtu_req(gattc_if, param->open.conn_id);
                }
            } else {
                ESP_LOGE(TAG, "Connect failed: %d", param->open.status);
                instance = find_button_by_addr(param->open.remote_bda);
                if (instance) {
                    instance->conn_state = FLIC2_CONN_STATE_FAILED;
                }
            }
            break;

        case ESP_GATTC_CFG_MTU_EVT:
            if (param->cfg_mtu.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "MTU configured: %d", param->cfg_mtu.mtu);
                instance = find_button_by_conn_id(param->cfg_mtu.conn_id);
                if (instance) {
                    instance->mtu = param->cfg_mtu.mtu;
                    // Start service discovery
                    esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
                }
            }
            break;

        case ESP_GATTC_SEARCH_RES_EVT:
            // Service found
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
                if (memcmp(param->search_res.srvc_id.uuid.uuid.uuid128,
                          FLIC2_SERVICE_UUID, 16) == 0) {
                    ESP_LOGI(TAG, "Found Flic2 service, start_handle=%d, end_handle=%d",
                             param->search_res.start_handle, param->search_res.end_handle);

                    instance = find_button_by_conn_id(param->search_res.conn_id);
                    if (instance) {
                        // Get characteristics
                        uint16_t count = 0;
                        esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
                            gattc_if, param->search_res.conn_id,
                            ESP_GATT_DB_CHARACTERISTIC,
                            param->search_res.start_handle,
                            param->search_res.end_handle,
                            0, &count);

                        if (status == ESP_GATT_OK && count > 0) {
                            esp_gattc_char_elem_t* chars = (esp_gattc_char_elem_t*)malloc(sizeof(esp_gattc_char_elem_t) * count);
                            if (chars) {
                                status = esp_ble_gattc_get_all_char(
                                    gattc_if, param->search_res.conn_id,
                                    param->search_res.start_handle,
                                    param->search_res.end_handle,
                                    chars, &count, 0);

                                if (status == ESP_GATT_OK) {
                                    for (int i = 0; i < count; i++) {
                                        if (chars[i].uuid.len == ESP_UUID_LEN_128) {
                                            if (memcmp(chars[i].uuid.uuid.uuid128,
                                                      FLIC2_CHAR_WRITE_UUID, 16) == 0) {
                                                instance->write_handle = chars[i].char_handle;
                                                ESP_LOGI(TAG, "Found write char, handle=%d",
                                                         chars[i].char_handle);
                                            }
                                            if (memcmp(chars[i].uuid.uuid.uuid128,
                                                      FLIC2_CHAR_NOTIFY_UUID, 16) == 0) {
                                                instance->notify_handle = chars[i].char_handle;
                                                ESP_LOGI(TAG, "Found notify char, handle=%d",
                                                         chars[i].char_handle);
                                            }
                                        }
                                    }
                                }
                                free(chars);
                            }
                        }
                    }
                }
            }
            break;

        case ESP_GATTC_SEARCH_CMPL_EVT:
            ESP_LOGI(TAG, "Service search complete");
            instance = find_button_by_conn_id(param->search_cmpl.conn_id);
            if (instance && instance->write_handle && instance->notify_handle) {
                instance->conn_state = FLIC2_CONN_STATE_REGISTERING_NOTIFY;

                // Get CCCD handle and register for notifications
                uint16_t count = 1;
                esp_gattc_descr_elem_t descr;
                esp_bt_uuid_t cccd_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
                };

                esp_gatt_status_t status = esp_ble_gattc_get_descr_by_char_handle(
                    gattc_if, param->search_cmpl.conn_id,
                    instance->notify_handle, cccd_uuid, &descr, &count);

                if (status == ESP_GATT_OK && count > 0) {
                    instance->cccd_handle = descr.handle;
                    esp_ble_gattc_register_for_notify(gattc_if,
                        instance->bd_addr, instance->notify_handle);
                }
            }
            break;

        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            if (param->reg_for_notify.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Registered for notifications");

                // Find instance by notify handle (remote_bda not available in ESP-IDF 5.x)
                instance = find_button_by_notify_handle(param->reg_for_notify.handle);
                if (instance) {
                    // Enable notifications by writing to CCCD
                    uint16_t notify_enable = 1;
                    esp_ble_gattc_write_char_descr(
                        gattc_if, instance->conn_id,
                        instance->cccd_handle,
                        sizeof(notify_enable),
                        (uint8_t*)&notify_enable,
                        ESP_GATT_WRITE_TYPE_RSP,
                        ESP_GATT_AUTH_REQ_NONE);
                }
            }
            break;

        case ESP_GATTC_WRITE_DESCR_EVT:
            if (param->write.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "CCCD written, starting Flic2 session");

                instance = find_button_by_conn_id(param->write.conn_id);
                if (instance) {
                    instance->conn_state = FLIC2_CONN_STATE_CONNECTED;

                    // Start Flic2 protocol session
                    double current_time = flic2_platform_get_steady_time();
                    flic2_start(&instance->button, current_time, instance->mtu);

                    process_flic2_events(instance);

                    // Notify callback
                    if (manager.callbacks.on_connection_change) {
                        manager.callbacks.on_connection_change(instance->bd_addr, true);
                    }
                }
            }
            break;

        case ESP_GATTC_NOTIFY_EVT:
            // Incoming notification from Flic2 button
            instance = find_button_by_conn_id(param->notify.conn_id);
            if (instance && param->notify.handle == instance->notify_handle) {
                double utc_time = flic2_platform_get_utc_time();
                double steady_time = flic2_platform_get_steady_time();

                xSemaphoreTake(manager.mutex, portMAX_DELAY);
                flic2_on_incoming_packet(&instance->button, utc_time, steady_time,
                                          param->notify.value, param->notify.value_len);
                process_flic2_events(instance);
                xSemaphoreGive(manager.mutex);
            }
            break;

        case ESP_GATTC_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Disconnected, reason=%d", param->disconnect.reason);

            instance = find_button_by_addr(param->disconnect.remote_bda);
            if (instance) {
                flic2_on_disconnected(&instance->button);
                instance->conn_state = FLIC2_CONN_STATE_DISCONNECTED;
                instance->timer_active = false;

                if (manager.callbacks.on_connection_change) {
                    manager.callbacks.on_connection_change(instance->bd_addr, false);
                }
            }
            break;

        default:
            break;
    }
}

// ============================================================================
// Flic2 Event Processing
// ============================================================================

static void process_flic2_events(flic2_button_instance_t* instance) {
    struct Flic2Event event;
    double utc_time = flic2_platform_get_utc_time();
    double steady_time = flic2_platform_get_steady_time();

    while (flic2_get_next_event(&instance->button, utc_time, steady_time, &event, true)) {
        // Handle DB updates
        if (event.db_update.type != FLIC2_DB_UPDATE_TYPE_NONE) {
            handle_db_update(instance, &event.db_update);
        }

        switch (event.type) {
            case FLIC2_EVENT_TYPE_SET_TIMER: {
                double delay = event.event.set_timer.absolute_time - steady_time;
                if (delay < 0) delay = 0;
                instance->timer_target_us = esp_timer_get_time() + (int64_t)(delay * 1000000);
                instance->timer_active = true;
                break;
            }

            case FLIC2_EVENT_TYPE_ABORT_TIMER:
                instance->timer_active = false;
                break;

            case FLIC2_EVENT_TYPE_OUTGOING_PACKET:
                // Write to button
                esp_ble_gattc_write_char(
                    instance->gattc_if, instance->conn_id,
                    instance->write_handle,
                    event.event.outgoing_packet.len,
                    event.event.outgoing_packet.data,
                    ESP_GATT_WRITE_TYPE_NO_RSP,
                    ESP_GATT_AUTH_REQ_NONE);
                break;

            case FLIC2_EVENT_TYPE_PAIRED:
                ESP_LOGI(TAG, "Paired! Name: %s, Serial: %s",
                         event.event.paired.name.value,
                         event.event.paired.serial_number);
                instance->paired = true;
                instance->conn_state = FLIC2_CONN_STATE_SESSION_ACTIVE;

                if (manager.callbacks.on_paired) {
                    manager.callbacks.on_paired(instance->bd_addr,
                                                 event.event.paired.name.value,
                                                 event.event.paired.serial_number);
                }
                break;

            case FLIC2_EVENT_TYPE_UNPAIRED:
                ESP_LOGI(TAG, "Button unpaired (factory reset)");
                instance->paired = false;

                if (manager.callbacks.on_unpaired) {
                    manager.callbacks.on_unpaired(instance->bd_addr);
                }
                break;

            case FLIC2_EVENT_TYPE_PAIRING_FAILED:
                ESP_LOGE(TAG, "Pairing failed: code=%d, subcode=%d",
                         event.event.pairing_failed.error_code,
                         event.event.pairing_failed.subcode);
                instance->conn_state = FLIC2_CONN_STATE_FAILED;
                break;

            case FLIC2_EVENT_TYPE_SESSION_FAILED:
                ESP_LOGE(TAG, "Session failed: code=%d, subcode=%d",
                         event.event.session_failed.error_code,
                         event.event.session_failed.subcode);
                break;

            case FLIC2_EVENT_TYPE_REAUTHENTICATED:
                ESP_LOGI(TAG, "Session established (quick verify)");
                instance->conn_state = FLIC2_CONN_STATE_SESSION_ACTIVE;
                break;

            case FLIC2_EVENT_TYPE_BUTTON_EVENT:
                ESP_LOGI(TAG, "Button event: class=%d, type=%d, queued=%d",
                         event.event.button_event.event_class,
                         event.event.button_event.event_type,
                         event.event.button_event.was_queued);

                if (manager.callbacks.on_button_event) {
                    manager.callbacks.on_button_event(
                        instance->bd_addr,
                        event.event.button_event.event_type,
                        event.event.button_event.event_class,
                        event.event.button_event.was_queued);
                }
                break;

            case FLIC2_EVENT_TYPE_BATTERY_VOLTAGE_UPDATED:
                ESP_LOGI(TAG, "Battery: %d mV",
                         event.event.battery_voltage_updated.millivolt);

                if (manager.callbacks.on_battery_update) {
                    manager.callbacks.on_battery_update(
                        instance->bd_addr,
                        event.event.battery_voltage_updated.millivolt);
                }
                break;

            case FLIC2_EVENT_TYPE_NAME_UPDATED:
                ESP_LOGI(TAG, "Name updated: %s", event.event.name_updated.name);
                break;

            case FLIC2_EVENT_TYPE_CHECK_FIRMWARE_REQUEST:
                // We don't support firmware updates in this implementation
                ESP_LOGI(TAG, "Firmware check requested (not implemented)");
                flic2_on_downloaded_firmware(&instance->button, utc_time, steady_time,
                                             FLIC2_FIRMWARE_DOWNLOAD_RESULT_FAILED, NULL, 0);
                break;

            default:
                break;
        }
    }
}

static void handle_db_update(flic2_button_instance_t* instance, struct Flic2DbUpdate* db_update) {
    switch (db_update->type) {
        case FLIC2_DB_UPDATE_TYPE_ADD:
        case FLIC2_DB_UPDATE_TYPE_UPDATE:
            flic2_storage_save(instance->bd_addr, &db_update->fields);
            break;

        case FLIC2_DB_UPDATE_TYPE_DELETE:
            flic2_storage_delete(instance->bd_addr);
            break;

        default:
            break;
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

static flic2_button_instance_t* find_button_by_addr(const uint8_t bd_addr[6]) {
    for (int i = 0; i < manager.button_count; i++) {
        if (memcmp(manager.buttons[i].bd_addr, bd_addr, 6) == 0) {
            return &manager.buttons[i];
        }
    }
    return NULL;
}

static flic2_button_instance_t* find_button_by_conn_id(uint16_t conn_id) {
    for (int i = 0; i < manager.button_count; i++) {
        if (manager.buttons[i].conn_id == conn_id &&
            manager.buttons[i].conn_state != FLIC2_CONN_STATE_DISCONNECTED) {
            return &manager.buttons[i];
        }
    }
    return NULL;
}

static flic2_button_instance_t* find_button_by_notify_handle(uint16_t handle) {
    for (int i = 0; i < manager.button_count; i++) {
        if (manager.buttons[i].notify_handle == handle &&
            manager.buttons[i].conn_state != FLIC2_CONN_STATE_DISCONNECTED) {
            return &manager.buttons[i];
        }
    }
    return NULL;
}

static flic2_button_instance_t* allocate_button(const uint8_t bd_addr[6]) {
    // Check if already exists
    flic2_button_instance_t* instance = find_button_by_addr(bd_addr);
    if (instance) {
        return instance;
    }

    // Allocate new
    if (manager.button_count >= FLIC2_MAX_BUTTONS) {
        return NULL;
    }

    instance = &manager.buttons[manager.button_count++];
    memset(instance, 0, sizeof(flic2_button_instance_t));
    memcpy(instance->bd_addr, bd_addr, 6);
    instance->conn_state = FLIC2_CONN_STATE_DISCONNECTED;
    instance->mtu = 23;  // Default MTU

    return instance;
}
