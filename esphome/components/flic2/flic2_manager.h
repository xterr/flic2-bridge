#ifndef FLIC2_MANAGER_H
#define FLIC2_MANAGER_H

#include "flic2_esp32_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t flic2_manager_init(const flic2_callbacks_t* callbacks);
esp_err_t flic2_manager_deinit(void);

esp_err_t flic2_manager_pair(const uint8_t bd_addr[6]);
esp_err_t flic2_manager_connect(const uint8_t bd_addr[6]);
esp_err_t flic2_manager_disconnect(const uint8_t bd_addr[6]);
esp_err_t flic2_manager_unpair(const uint8_t bd_addr[6]);

int flic2_manager_get_paired(uint8_t (*bd_addrs)[6], int max_count);
flic2_button_instance_t* flic2_manager_get_button(const uint8_t bd_addr[6]);

void flic2_manager_loop(void);

int flic2_manager_get_connected_count(void);
bool flic2_manager_is_connected(const uint8_t bd_addr[6]);
bool flic2_manager_get_name(const uint8_t bd_addr[6], char* name_buf);
uint16_t flic2_manager_get_battery_mv(const uint8_t bd_addr[6]);

#ifdef __cplusplus
}
#endif

#endif
