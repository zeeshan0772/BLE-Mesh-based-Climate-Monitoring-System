
#include <stdio.h>
#include <string.h>
#include <sdkconfig.h>
#include "BLE_stack_init.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_ble_mesh_defs.h"

#define TAG "MESH_INIT"

void ble_mesh_get_dev_uuid(uint8_t *dev_uuid)
{
    if (dev_uuid == NULL) {
        //ESP_LOGE(TAG, "%s, Invalid device uuid", __func__);
        return;
    }

    /* Copy device address to the device uuid with offset equals to 2 here.
     * The first two bytes is used for matching device uuid by Provisioner.
     * And using device address here is to avoid using the same device uuid
     * by different unprovisioned devices.
     */

     /* esp_bt_dev_get_address() function returns the address of bluetooth device
     and this address is copied to the memory location of dev_uuid vaiable
     */
    memcpy(dev_uuid + 2, esp_bt_dev_get_address(), BD_ADDR_LEN);
}

esp_err_t BLE_stack_init(void)
{
    esp_err_t ret;
    // release the memory held by classic bluetooth
    // returns: success or failed
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // configure the default bluetooth controller 
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    /* 
    Initialize BT controller to allocate task and other resource. This function should be called only once, before
    any other BT functions are called.

    Parameters cfg - Initial configuration of BT controller. Different from previous version, there's 
    a mode and some connection configuration in 'cfg' to configure controller work mode and
    allocate the resource which is needed.
    Returns ESP_OK - success, other - failed
    */
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        //ESP_LOGE(TAG, "%s initialize controller failed", __func__);
        return ret;
    }

    // Enable BT controller.
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        //ESP_LOGE(TAG, "%s enable controller failed", __func__);
        return ret;
    }

    // Init and alloc the resource for bluetooth, must be prior to every bluetooth stuff.
    ret = esp_bluedroid_init();
    if (ret) {
        //ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return ret;
    }

    // Enable bluetooth, must after esp_bluedroid_init().
    ret = esp_bluedroid_enable();
    if (ret) {
        //ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return ret;
    }

    return ret;
}