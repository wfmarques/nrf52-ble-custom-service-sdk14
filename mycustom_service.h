#ifndef MYCUSTOM_SERVICE_H__
#define MYCUSTOM_SERVICE_H__

#include <stdint.h>
#include "ble_srv_common.h"

#define BLE_UUID_CUSTOM_SERVICE          0x1110
#define BLE_UUID_CUSTOM_CHAR1            0x0001
#define BLE_UUID_CUSTOM_CHAR2            0x0002
#define CHARACTERISTIC_SIZE 			 20


uint32_t mycustom_service_init(void);
uint32_t mycustom_service_update_data(uint16_t conn_handle,uint8_t *new_data);

#endif

