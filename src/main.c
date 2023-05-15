#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"

// contains bluetooth stack initialization code
#include "BLE_stack_init.h"
#include "sensor_driver.h"

#define NODE_CID 0x02E5
#define TAG "Meshsetup"

// sensor descriptor related information
// property id for present outdoor ambient temperature
#define HUM_SENSOR_PROP_ID 0x0056
#define TEMP_SENSOR_PROP_ID 0x005B

// property id 0x0076 doesn't works
//#define HUM_SENSOR_PROP_ID 0x0076


#define SENSOR_POSITIVE_TOLERANCE ESP_BLE_MESH_SENSOR_UNSPECIFIED_POS_TOLERANCE
#define SENSOR_NEGATIVE_TOLERANCE ESP_BLE_MESH_SENSOR_UNSPECIFIED_NEG_TOLERANCE
#define SENSOR_SAMPLE_FUNCTION ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED
#define SENSOR_MEASURE_PERIOD ESP_BLE_MESH_SENSOR_NOT_APPL_MEASURE_PERIOD
#define SENSOR_UPDATE_INTERVAL ESP_BLE_MESH_SENSOR_NOT_APPL_UPDATE_INTERVAL


static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};

// Define a static net_buf_simple variable.
// This is a helper macro which is used to define a static net_buf_simple object.
// Parameters   NET_BUF_SIMPLE_DEFINE_STATIC(_name, _size)
// _name: Name of the net_buf_simple object.
// _size: Maximum data storage for the buffer.

NET_BUF_SIMPLE_DEFINE_STATIC(humidity_sensor_data, 1);
NET_BUF_SIMPLE_DEFINE_STATIC(temperature_sensor_data, 1);

// parameters of sensor state
static esp_ble_mesh_sensor_state_t sensor_state[2] = {
    [0] = {
        .sensor_property_id = HUM_SENSOR_PROP_ID,
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 0, /* 0 represents the length is 1 */
        .sensor_data.raw_value = &humidity_sensor_data,
    },
    [1] = {
        .sensor_property_id = TEMP_SENSOR_PROP_ID,
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 0, // 0 represents the length is 1
        .sensor_data.raw_value = &temperature_sensor_data,
    },
    };

/*
    Define a model publication context 
    20 octets is large enough to hold two Sensor Descriptor state values. */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 20, ROLE_NODE);

static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .state_count = ARRAY_SIZE(sensor_state),
    .states = sensor_state,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_setup_pub, 20, ROLE_NODE);
static esp_ble_mesh_sensor_setup_srv_t sensor_setup_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .state_count = ARRAY_SIZE(sensor_state),
    .states = sensor_state,
};

// Configuration Server
static esp_ble_mesh_cfg_srv_t configuration_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
    //.gatt_proxy = ESP_BLE_MESH_GATT_PROXY_DISABLED,
    .default_ttl = 10,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20), // Total of 3 transmissions after every 20ms
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

// Root models
// all the server/client models are initialized here
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&configuration_server), // define a configuration server model
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
    ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server),
};

// Element definition
static esp_ble_mesh_elem_t elements[] = {
    // Helper to define a BLE Mesh element within an array.
    // In case the element has no SIG or Vendor models, the helper
    // macro ESP_BLE_MESH_MODEL_NONE can be given instead.
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

// Node composition
static esp_ble_mesh_comp_t composition = {
    .cid = NODE_CID,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

// contains provisioning related data
static esp_ble_mesh_prov_t prov = {
    .uuid = dev_uuid,
};

esp_ble_mesh_model_t *get_sensor_server_model(void)
{
    return &root_models[1];
}


static esp_err_t initialize_mesh(void)
{
    esp_err_t error_code = ESP_OK;
    ble_mesh_get_dev_uuid(dev_uuid);
    error_code = esp_ble_mesh_init(&prov, &composition);
    error_code = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_GATT | ESP_BLE_MESH_PROV_ADV);
    return error_code;
}

static void mesh_provisioning_callback(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "Provisioning complete");
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        // All provisioning information in this node will be deleted and the node needs to be reprovisioned. The
        // API function esp_ble_mesh_node_prov_enable() needs to be called to start a new provisioning procedure.
        esp_ble_mesh_node_local_reset();
    default:
        break;
    }
}

static void config_server_callback(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
    {
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "config: app key added");
            break;
        default:
            break;
        }
    }
}

/**
 * : after variable name is used for specifying the number of bit fields to use for variable
 * ((packed)) means it will use the smallest possible space for struct Ball - i.e. it will cram fields together without padding
 */
struct sensor_descriptor
{
    uint16_t sensor_prop_id;
    uint32_t pos_tolerance : 12,
        neg_tolerance : 12,
        sample_func : 8;
    uint8_t measure_period;
    uint8_t update_interval;
} __attribute__((packed));


/**
 * @brief Sends sensor descriptor state data in response to sensor descriptor get message
 * Sensor Descriptor Status message ==> Sensor Descriptor Get message
 * @param param
 * Sensor Server Model callback parameters
 * Public Members of esp_ble_mesh_sensor_server_cb_param_t: 
 *      esp_ble_mesh_model_t *model - Pointer to Sensor Server Models 
 *      esp_ble_mesh_msg_ctx_t ctx - Context of the received messages
 *      esp_ble_mesh_sensor_server_cb_value_t value - Value of the received Sensor Messages 
 * 
 * @details
 * Message Types:
 * 1) Sensor Descriptor Get 
 * Sensor Descriptor Get is an acknowledged message used to get the Sensor Descriptor state of all  
 * sensors within an element.
 * 
 * 2) Sensor Descriptor Status 
 * The Sensor Descriptor Status is an unacknowledged message used to report a sequence of the 
 * Sensor Descriptor states of an element.
 */
static void ble_mesh_send_sensor_descriptor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct sensor_descriptor descriptor = {0}; // initialize all bits to zero
    uint8_t *status = NULL;     // pointer to memory allocated to descriptor state struct
    uint16_t length = 0;    // length of sensor descriptor state
    esp_err_t err;
    int i;


    status = calloc(1, ARRAY_SIZE(sensor_state) * ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
    if (!status)
    {
        ESP_LOGE(TAG, "No memory for sensor descriptor status!");
        return;
    }

    if (param->value.get.sensor_descriptor.op_en == false)
    {
        /* Mesh Model Spec:
         * Upon receiving a Sensor Descriptor Get message with the Property ID field
         * omitted, the Sensor Server shall respond with a Sensor Descriptor Status
         * message containing the Sensor Descriptor states for all sensors within the
         * Sensor Server.
         */
        for (i = 0; i < ARRAY_SIZE(sensor_state); i++)
        {
            descriptor.sensor_prop_id = sensor_state[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_state[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_state[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_state[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_state[i].descriptor.measure_period;
            descriptor.update_interval = sensor_state[i].descriptor.update_interval;

            memcpy(status + length, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length += ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
        }
        goto send;
    }
    for (i = 0; i < ARRAY_SIZE(sensor_state); i++)
    {
        /* Send descriptor state when the get message contains the valid sensor property ID */
        if (param->value.get.sensor_descriptor.property_id == sensor_state[i].sensor_property_id)
        {
            descriptor.sensor_prop_id = sensor_state[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_state[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_state[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_state[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_state[i].descriptor.measure_period;
            descriptor.update_interval = sensor_state[i].descriptor.update_interval;
            memcpy(status, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length = ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
            goto send;
        }
    }
    /* Mesh Model Spec:
     * When a Sensor Descriptor Get message that identifies a sensor descriptor
     * property that does not exist on the element, the Descriptor field shall
     * contain the requested Property ID value and the other fields of the Sensor
     * Descriptor state shall be omitted.
     */
    memcpy(status, &param->value.get.sensor_descriptor.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Descriptor", status, length);    // Log a buffer of hex bytes at Info level.
    
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS, length, status);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Descriptor Status");
    }
    free(status);   // release the memory held by descriptor state structure
}

static void ble_mesh_send_sensor_cadence_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    /* Sensor Cadence state is not supported currently. */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_STATUS,
                                             ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
                                             (uint8_t *)&param->value.get.sensor_cadence.property_id);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Cadence Status");
    }
}

static void ble_mesh_send_sensor_settings_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    /* Sensor Setting state is not supported currently. */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS,
                                             ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
                                             (uint8_t *)&param->value.get.sensor_settings.property_id);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Settings Status");
    }
}

struct sensor_setting
{
    uint16_t sensor_prop_id;
    uint16_t sensor_setting_prop_id;
} __attribute__((packed));

static void ble_mesh_send_sensor_setting_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct sensor_setting setting = {0};
    esp_err_t err;

    /* Mesh Model Spec:
     * If the message is sent as a response to the Sensor Setting Get message or
     * a Sensor Setting Set message with an unknown Sensor Property ID field or
     * an unknown Sensor Setting Property ID field, the Sensor Setting Access
     * field and the Sensor Setting Raw field shall be omitted.
     */

    setting.sensor_prop_id = param->value.get.sensor_setting.property_id;
    setting.sensor_setting_prop_id = param->value.get.sensor_setting.setting_property_id;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_STATUS,
                                             sizeof(setting), (uint8_t *)&setting);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Setting Status");
    }
}

static uint16_t ble_mesh_get_sensor_data(esp_ble_mesh_sensor_state_t *state, uint8_t *data)
{
    uint8_t mpid_len = 0, data_len = 0;
    uint32_t mpid = 0;

    if (state == NULL || data == NULL)
    {
        ESP_LOGE(TAG, "%s, Invalid parameter", __func__);
        return 0;
    }

    if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
    {
        /* For zero-length sensor data, the length is 0x7F, and the format is Format B. */
        mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
        mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        data_len = 0;
    }
    else
    {
        if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A)
        {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN;
        }
        else
        {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
        data_len = state->sensor_data.length + 1;
    }

    // ???
    memcpy(data, &mpid, mpid_len);
    memcpy(data + mpid_len, state->sensor_data.raw_value->data, data_len);

    return (mpid_len + data_len);
}


static void ble_mesh_send_sensor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t buf_size = 0;
    uint16_t length = 0;
    uint32_t mpid = 0;
    esp_err_t err;
    int i;

    /**
     * Sensor Data state from Mesh Model Spec
     * |--------Field--------|-Size (octets)-|------------------------Notes-------------------------|
     * |----Property ID 1----|-------2-------|--ID of the 1st device property of the sensor---------|
     * |-----Raw Value 1-----|----variable---|--Raw Value field defined by the 1st device property--|
     * |----Property ID 2----|-------2-------|--ID of the 2nd device property of the sensor---------|
     * |-----Raw Value 2-----|----variable---|--Raw Value field defined by the 2nd device property--|
     * | ...... |
     * |----Property ID n----|-------2-------|--ID of the nth device property of the sensor---------|
     * |-----Raw Value n-----|----variable---|--Raw Value field defined by the nth device property--|
     */

    // find the buffer size for sensor status
    for (i = 0; i < ARRAY_SIZE(sensor_state); i++)
    {
        esp_ble_mesh_sensor_state_t *state = &sensor_state[i];
        if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
        {
            buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        else
        {
            /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
            if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A)
            {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN + state->sensor_data.length + 1;
            }
            else
            {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN + state->sensor_data.length + 1;
            }
        }
    }

    status = calloc(1, buf_size);
    if (!status)
    {
        ESP_LOGE(TAG, "No memory for sensor status!");
        return;
    }

    if (param->value.get.sensor_data.op_en == false)
    {
        /* Mesh Model Spec:
         * If the message is sent as a response to the Sensor Get message, and if the
         * Property ID field of the incoming message is omitted, the Marshalled Sensor
         * Data field shall contain data for all device properties within a sensor.
         */
        for (i = 0; i < ARRAY_SIZE(sensor_state); i++)
        {
            length += ble_mesh_get_sensor_data(&sensor_state[i], status + length);
        }
        goto send;
    }

    /* Mesh Model Spec:
     * Otherwise, the Marshalled Sensor Data field shall contain data for the requested
     * device property only.
     */
    for (i = 0; i < ARRAY_SIZE(sensor_state); i++)
    {
        if (param->value.get.sensor_data.property_id == sensor_state[i].sensor_property_id)
        {
            length = ble_mesh_get_sensor_data(&sensor_state[i], status);
            goto send;
        }
    }

    /* Mesh Model Spec:
     * Or the Length shall represent the value of zero and the Raw Value field shall
     * contain only the Property ID if the requested device property is not recognized
     * by the Sensor Server.
     */
    mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN,
                                                  param->value.get.sensor_data.property_id);
    memcpy(status, &mpid, ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);
    length = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Data", status, length);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, length, status);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Status");
    }
    free(status);
}

static void publish_data()
{
    uint8_t *status = NULL;
    uint16_t buf_size = 0;
    uint16_t length = 0;
    esp_err_t err;
    int i;

    // find the buffer size for sensor status
    for (i = 0; i < ARRAY_SIZE(sensor_state); i++)
    {
        esp_ble_mesh_sensor_state_t *state = &sensor_state[i];
        if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
        {
            buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        else
        {
            /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
            if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A)
            {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN + state->sensor_data.length + 1;
            }
            else
            {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN + state->sensor_data.length + 1;
            }
        }
    }        

    status = calloc(1, buf_size);
 
    if (!status)
    {
        ESP_LOGE(TAG, "No memory for sensor status!");
        return;
    }

    for (i = 0; i < ARRAY_SIZE(sensor_state); i++)
    {
        length += ble_mesh_get_sensor_data(&sensor_state[i], status + length);
    }

    ESP_LOG_BUFFER_HEX("Published Sensor Data", status, length);

    esp_ble_mesh_model_t *model = get_sensor_server_model();
    err = esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, length, status, ROLE_NODE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to publish Sensor Status");
    }
    free(status);
}




static void ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                              esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Sensor server, event %d, src 0x%04x, dst 0x%04x, model_id 0x%04x",
             event, param->ctx.addr, param->ctx.recv_dst, param->model->model_id);

    switch (event)
    {
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT:
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET");
            ble_mesh_send_sensor_descriptor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET");
            ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            ble_mesh_send_sensor_settings_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            ble_mesh_send_sensor_setting_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_GET");
            ble_mesh_send_sensor_status(param);
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Get opcode 0x%04x", param->ctx.recv_op);
            return;
        }
        break;
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT:
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET");
            ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK");
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET");
            ble_mesh_send_sensor_setting_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK");
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Set opcode 0x%04x", param->ctx.recv_op);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Server event %d", event);
        break;
    }
}

void app_main()
{

    nvs_flash_erase();
    nvs_flash_init();


    BLE_stack_init();

    esp_ble_mesh_register_prov_callback(mesh_provisioning_callback);
    esp_ble_mesh_register_config_server_callback(config_server_callback);
    esp_ble_mesh_register_sensor_server_callback(ble_mesh_sensor_server_cb);
    initialize_mesh();
    
    net_buf_simple_add_u8(&humidity_sensor_data, 2*humidity);
    net_buf_simple_add_u8(&temperature_sensor_data, 2*temperature);
    
    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        get_temperature_and_humidity();
        printf("Humidity: %f%% Temp: %fC\n", humidity, temperature);

        
        net_buf_simple_pull(&humidity_sensor_data, sizeof(uint8_t));
        net_buf_simple_pull(&temperature_sensor_data, sizeof(uint8_t));

        net_buf_simple_add_u8(&humidity_sensor_data, 2*humidity);
        net_buf_simple_add_u8(&temperature_sensor_data, 2*temperature);
        
        publish_data();
    }
}