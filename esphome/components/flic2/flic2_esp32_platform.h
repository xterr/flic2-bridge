/**
 * Flic 2 ESP32 Platform Abstraction Layer
 *
 * Provides ESP32-specific implementations for:
 * - BLE GATT client operations
 * - Time functions (monotonic clock, wall clock)
 * - Cryptographically secure random number generation
 * - NVS persistent storage for pairing data
 */

#ifndef FLIC2_ESP32_PLATFORM_H
#define FLIC2_ESP32_PLATFORM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "flic2.h"

#ifdef __cplusplus
extern "C" {
#endif

// Flic 2 GATT Service UUID: 00420000-8F59-4420-870D-84F3B617E493
#define FLIC2_SERVICE_UUID_128 {0x93, 0xE4, 0x17, 0xB6, 0xF3, 0x84, 0x0D, 0x87, \
                                 0x20, 0x44, 0x59, 0x8F, 0x00, 0x00, 0x42, 0x00}

// Write characteristic UUID: 00420001-8F59-4420-870D-84F3B617E493
#define FLIC2_CHAR_WRITE_UUID_128 {0x93, 0xE4, 0x17, 0xB6, 0xF3, 0x84, 0x0D, 0x87, \
                                    0x20, 0x44, 0x59, 0x8F, 0x01, 0x00, 0x42, 0x00}

// Notify characteristic UUID: 00420002-8F59-4420-870D-84F3B617E493
#define FLIC2_CHAR_NOTIFY_UUID_128 {0x93, 0xE4, 0x17, 0xB6, 0xF3, 0x84, 0x0D, 0x87, \
                                     0x20, 0x44, 0x59, 0x8F, 0x02, 0x00, 0x42, 0x00}

// Maximum number of buttons supported
#define FLIC2_MAX_BUTTONS 5

// Button connection state
typedef enum {
    FLIC2_CONN_STATE_DISCONNECTED,
    FLIC2_CONN_STATE_CONNECTING,
    FLIC2_CONN_STATE_DISCOVERING_SERVICES,
    FLIC2_CONN_STATE_REGISTERING_NOTIFY,
    FLIC2_CONN_STATE_CONNECTED,
    FLIC2_CONN_STATE_SESSION_ACTIVE,
    FLIC2_CONN_STATE_FAILED
} flic2_conn_state_t;

// Button instance with connection info
typedef struct {
    struct Flic2Button button;
    uint8_t bd_addr[6];
    flic2_conn_state_t conn_state;
    uint16_t conn_id;
    uint16_t gattc_if;
    uint16_t write_handle;
    uint16_t notify_handle;
    uint16_t cccd_handle;
    uint16_t mtu;
    bool paired;
    bool timer_active;
    int64_t timer_target_us;
} flic2_button_instance_t;

// Callback types for ESPHome integration
typedef void (*flic2_button_event_cb_t)(uint8_t* bd_addr,
                                        enum Flic2EventButtonEventType event_type,
                                        enum Flic2EventButtonEventClass event_class,
                                        bool was_queued);
typedef void (*flic2_paired_cb_t)(uint8_t* bd_addr, const char* name, const char* serial);
typedef void (*flic2_unpaired_cb_t)(uint8_t* bd_addr);
typedef void (*flic2_battery_cb_t)(uint8_t* bd_addr, uint16_t millivolt);
typedef void (*flic2_connection_cb_t)(uint8_t* bd_addr, bool connected);

// Callback registration
typedef struct {
    flic2_button_event_cb_t on_button_event;
    flic2_paired_cb_t on_paired;
    flic2_unpaired_cb_t on_unpaired;
    flic2_battery_cb_t on_battery_update;
    flic2_connection_cb_t on_connection_change;
} flic2_callbacks_t;

// ============================================================================
// Time Functions
// ============================================================================

/**
 * Get current monotonic time in seconds (steady clock).
 * Uses esp_timer_get_time() which provides microsecond resolution.
 */
double flic2_platform_get_steady_time(void);

/**
 * Get current UNIX timestamp in seconds (wall clock).
 * Uses system time if available, otherwise returns 0.
 */
double flic2_platform_get_utc_time(void);

// ============================================================================
// Random Number Generation
// ============================================================================

/**
 * Fill buffer with cryptographically secure random bytes.
 * Uses esp_fill_random() from ESP-IDF.
 */
void flic2_platform_get_random(uint8_t* buf, size_t len);

// ============================================================================
// NVS Storage Functions
// ============================================================================

/**
 * Initialize NVS storage for Flic2 pairing data.
 */
esp_err_t flic2_storage_init(void);

/**
 * Save button pairing data to NVS.
 */
esp_err_t flic2_storage_save(const uint8_t bd_addr[6], const struct Flic2DbData* data);

/**
 * Load button pairing data from NVS.
 * Returns ESP_OK if found, ESP_ERR_NOT_FOUND if not paired.
 */
esp_err_t flic2_storage_load(const uint8_t bd_addr[6], struct Flic2DbData* data);

/**
 * Delete button pairing data from NVS.
 */
esp_err_t flic2_storage_delete(const uint8_t bd_addr[6]);

/**
 * List all paired button addresses.
 * Returns number of paired buttons found.
 */
int flic2_storage_list_paired(uint8_t (*bd_addrs)[6], int max_count);

// ============================================================================
// BLE Functions
// ============================================================================

/**
 * Initialize BLE subsystem for Flic2.
 */
esp_err_t flic2_ble_init(void);

/**
 * Deinitialize BLE subsystem.
 */
esp_err_t flic2_ble_deinit(void);

/**
 * Start scanning for Flic 2 buttons.
 * Buttons in pairing mode advertise the Flic2 service UUID.
 */
esp_err_t flic2_ble_start_scan(uint32_t duration_seconds);

/**
 * Stop scanning.
 */
esp_err_t flic2_ble_stop_scan(void);

/**
 * Connect to a Flic 2 button by address.
 */
esp_err_t flic2_ble_connect(const uint8_t bd_addr[6]);

/**
 * Disconnect from a button.
 */
esp_err_t flic2_ble_disconnect(const uint8_t bd_addr[6]);

/**
 * Write data to the Flic 2 write characteristic (Write Without Response).
 */
esp_err_t flic2_ble_write(flic2_button_instance_t* instance, const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // FLIC2_ESP32_PLATFORM_H
