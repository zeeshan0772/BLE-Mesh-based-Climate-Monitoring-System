
#ifndef _BLE_MESH_INIT_H_
#define _BLE_MESH_INIT_H_

#include "esp_err.h"

void ble_mesh_get_dev_uuid(uint8_t *dev_uuid);

esp_err_t BLE_stack_init(void);

#endif
