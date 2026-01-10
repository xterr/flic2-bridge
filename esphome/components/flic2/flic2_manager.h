/**
 * Flic 2 Manager
 *
 * High-level manager for Flic 2 buttons on ESP32.
 * Handles BLE operations, button lifecycle, and event dispatching.
 */

#ifndef FLIC2_MANAGER_H
#define FLIC2_MANAGER_H

#include "flic2_esp32_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the Flic2 manager.
 * Must be called before any other manager functions.
 *
 * @param callbacks Callback functions for events (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t flic2_manager_init(const flic2_callbacks_t* callbacks);

/**
 * Deinitialize the Flic2 manager.
 */
esp_err_t flic2_manager_deinit(void);

/**
 * Start scanning for Flic 2 buttons in pairing mode.
 *
 * @param duration_seconds Scan duration (0 = indefinite)
 * @return ESP_OK on success
 */
esp_err_t flic2_manager_start_scan(uint32_t duration_seconds);

/**
 * Stop scanning.
 */
esp_err_t flic2_manager_stop_scan(void);

/**
 * Pair with a Flic 2 button.
 * The button must be in pairing mode (hold 7+ seconds until LED flashes).
 *
 * @param bd_addr Bluetooth device address
 * @return ESP_OK if pairing started
 */
esp_err_t flic2_manager_pair(const uint8_t bd_addr[6]);

/**
 * Connect to an already paired button.
 *
 * @param bd_addr Bluetooth device address
 * @return ESP_OK if connection started
 */
esp_err_t flic2_manager_connect(const uint8_t bd_addr[6]);

/**
 * Disconnect from a button.
 *
 * @param bd_addr Bluetooth device address
 * @return ESP_OK on success
 */
esp_err_t flic2_manager_disconnect(const uint8_t bd_addr[6]);

/**
 * Unpair a button (remove from storage).
 *
 * @param bd_addr Bluetooth device address
 * @return ESP_OK on success
 */
esp_err_t flic2_manager_unpair(const uint8_t bd_addr[6]);

/**
 * Get list of paired button addresses.
 *
 * @param bd_addrs Array to fill with addresses
 * @param max_count Maximum number of addresses to return
 * @return Number of paired buttons
 */
int flic2_manager_get_paired(uint8_t (*bd_addrs)[6], int max_count);

/**
 * Get button instance by address.
 *
 * @param bd_addr Bluetooth device address
 * @return Button instance or NULL if not found
 */
flic2_button_instance_t* flic2_manager_get_button(const uint8_t bd_addr[6]);

/**
 * Process manager tasks (call periodically from main loop).
 * Handles timers, reconnections, etc.
 */
void flic2_manager_loop(void);

/**
 * Get number of connected buttons.
 */
int flic2_manager_get_connected_count(void);

/**
 * Check if a button is connected.
 */
bool flic2_manager_is_connected(const uint8_t bd_addr[6]);

/**
 * Get button name.
 *
 * @param bd_addr Bluetooth device address
 * @param name_buf Buffer to store name (min 24 bytes)
 * @return true if button found and has name
 */
bool flic2_manager_get_name(const uint8_t bd_addr[6], char* name_buf);

/**
 * Get button battery voltage in millivolts.
 *
 * @param bd_addr Bluetooth device address
 * @return Battery voltage in mV, or 0 if unknown
 */
uint16_t flic2_manager_get_battery_mv(const uint8_t bd_addr[6]);

#ifdef __cplusplus
}
#endif

#endif // FLIC2_MANAGER_H
