#include <string.h>

#include "../include/ble_conn_setup.h"
#include "../include/mc_general_defs.h"
#include "../include/mc_hid_defs.h"

#include "esp_system.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"


static const uint16_t GATT_SERVICE_DECL_UUID = 0x2800;
static const uint16_t GATT_CHAR_DECL_UUID = 0x2803;


// Profile tab groups all profiles used in the application
gatts_profile_inst motion_controller_profile_tab[MOTION_CONTROLLER_PROFILE_NUM] = {
    [HID_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    }
};

static uint16_t hid_handle_table[HID_IDX_NUM];  // Used to later store the handle of each gatt attribute

/* Characteristic property definitions */
static const esp_gatt_char_prop_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const esp_gatt_char_prop_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const esp_gatt_char_prop_t char_prop_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;


// Gatt table for the motion controller app, populated with HID service and its attributes ...
const esp_gatts_attr_db_t mc_gatt_db[HID_IDX_NUM] = {
    // HID Service.
    [IDX_HID_SERVICE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATT_SERVICE_DECL_UUID, ESP_GATT_PERM_READ, 
         sizeof(HID_SERVICE_UUID), sizeof(HID_SERVICE_UUID), (uint8_t *)HID_SERVICE_UUID}
    },

    /* 
    * Characteristic declaration: HID info.
    * Description: Holds general information about the ble peripheral such as version, operation intended country and certain flags.
    */
    [IDX_CHAR_HID_INFO_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATT_CHAR_DECL_UUID, ESP_GATT_PERM_READ, 
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_read}
    },
    /* 
    * Characteristic value: HID info.
    * Value type: uint8 array
    */
    [IDX_CHAR_HID_INFO_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&HID_INFO_UUID, ESP_GATT_PERM_READ, 
         sizeof(hid_info), sizeof(hid_info), (uint8_t *)hid_info}
    },

    /*
    * Characteristic declaration: HID report map.
    * Description: Holds parameters of the HID device in a logical way. 
    * Purpose: Inform the HID host of the HID device's components and possible functionality.
    */
    [IDX_CHAR_HID_REPORT_MAP_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATT_CHAR_DECL_UUID, ESP_GATT_PERM_READ, 
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_read}
    },
    /* 
    * Characteristic value: HID report map.
    * Value type: uint8 array
    */
    [IDX_CHAR_HID_REPORT_MAP_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&HID_REPORT_MAP_UUID, ESP_GATT_PERM_READ, 
         HID_REPORT_MAP_MAX_SIZE, sizeof(mc_hid_report_map), (uint8_t *)mc_hid_report_map}
    },

    /*
    * Characteristic declaration: HID control point.
    * Description: Setting this to 0x00 suspends the activity of the HID device. Setting it to 0x01 resumes the activity.
    */
    [IDX_CHAR_HID_CONTROL_POINT_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATT_CHAR_DECL_UUID, ESP_GATT_PERM_READ, 
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_write_nr}
    },
    /*
    * Characteristic value: HID control point.
    * Value Type: uint8
    */
    [IDX_CHAR_HID_CONTROL_POINT_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&HID_CONTROL_POINT_UUID, ESP_GATT_PERM_WRITE, 
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&hid_control_point}
    },

    /*
    * Characteristic declaration: HID report.
    * Description: Holds the actual values read from the HID device (e.g. holds the x and y axis movement of an HID mouse)
    */
    [IDX_CHAR_HID_REPORT_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATT_CHAR_DECL_UUID, ESP_GATT_PERM_READ, 
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_read_write}
    },
    /*
    * Characteristic value: HID report.
    * Value type: uint8 array
    */
    [IDX_CHAR_HID_REPORT_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&HID_REPORT_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(hid_report), sizeof(hid_report), (uint8_t *)hid_report}
    }
};


static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = false,                // Name set to false not to exceed the max advertising packet size of 31 bytes
    .include_txpower = true,
    .min_interval = 0x0006,                 // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010,                 // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,                   // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(HID_SERVICE_UUID),
    .p_service_uuid = HID_SERVICE_UUID,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    // ************************* Debugging ************************
    ESP_LOGI(GATTS_TAG, "Event triggered: %d", event);

    if (event == ESP_GATTS_WRITE_EVT) {
        ESP_LOGI(GATTS_TAG, "Write data = ");
        for (int i = 0; i < param->write.len; ++i) {
            ESP_LOGI(GATTS_TAG, "%u", *(param->write.value + i));
        }
    }
    // ************************************************************

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            motion_controller_profile_tab[HID_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < MOTION_CONTROLLER_PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == motion_controller_profile_tab[idx].gatts_if) {
                if (motion_controller_profile_tab[idx].gatts_cb) {
                    motion_controller_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    esp_err_t ret;

    switch(event) {
        /* 
        * Set a device name and configure advertising data following successfull application registration.
        * Also create an attribute table for the HID Service.
        */
        case ESP_GATTS_REG_EVT:
            ret = esp_ble_gap_set_device_name(DEVICE_NAME);
             if (ret != ESP_OK){
                ESP_LOGE(GATTS_TAG, "%s -> Error on line %d. Set device name failed! Error code: %d", __func__, __LINE__, ret);
                return;
             }
             ESP_LOGI(GATTS_TAG, "%s -> Set device name successful! LINE %d", __func__, __LINE__);

            ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret != ESP_OK){
                ESP_LOGE(GATTS_TAG, "%s -> Error on line %d. Config advertising data failed! Error code: %d", __func__, __LINE__, ret);
                return;
             }
             ESP_LOGI(GATTS_TAG, "%s -> Config advertising data successful! LINE %d", __func__, __LINE__);

            ret = esp_ble_gatts_create_attr_tab(mc_gatt_db, gatts_if, HID_IDX_NUM, HID_SERVICE_INST);
            if (ret != ESP_OK) {
                ESP_LOGE(GATTS_TAG, "%s -> Error on line %d. Creating gatt atribute table failed! Error code: %d", __func__, __LINE__, ret);
                return;
            }
            ESP_LOGI(GATTS_TAG, "%s -> Created gatt attribute table successfully! LINE %d", __func__, __LINE__);
            break;
            
        /* 
        * Check whether the attribute table has been created correctly.
        * Start the hid gatt service.
        */
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGI(GATTS_TAG, "The total number of handles = %d", param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
                return;
            }
            else if (param->add_attr_tab.num_handle != HID_IDX_NUM) {
                ESP_LOGE(GATTS_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HID_IDX_NUM(%d)", param->add_attr_tab.num_handle, HID_IDX_NUM);
                return;
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number of handles = %d", param->add_attr_tab.num_handle);

                memcpy(hid_handle_table, param->add_attr_tab.handles, sizeof(hid_handle_table));
                ESP_LOGI(GATTS_TAG, "HID handle table: ");
                for (int i = 0; i < param->add_attr_tab.num_handle; ++i) {
                    ESP_LOGI(GATTS_TAG, "%d", hid_handle_table[i]);
                }
                esp_ble_gatts_start_service(hid_handle_table[IDX_HID_SERVICE]);
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);

            // Update HID report's xy value and notify the BLE client
            motion_controller_profile_tab[HID_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
            hid_report[1] = 100; // X offset
            // hid_report[2] = 10; // Y offset
            esp_ble_gatts_send_indicate(gatts_if, param->connect.conn_id, hid_handle_table[IDX_CHAR_HID_REPORT_VAL],
                                        HID_REPORT_LEN, hid_report, false);
            break;

        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TAG, "Notification status = %d", param->conf.status);
            ESP_LOGI(GATTS_TAG, "Notification value length = %d", param->conf.len);

            for (int i = 0; i < param->conf.len; ++i) {
                ESP_LOGI(GATTS_TAG, "Byte%d = %u", i, *(param->conf.value + i));
            }
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret != ESP_OK) {
                ESP_LOGE(GATTS_TAG, "Device disconnected, Restarting advertising failed!");
            }
            ESP_LOGI(GATTS_TAG, "%s -> Device disconnected! Restarting advertising! LINE %d", __func__, __LINE__);
            break;

        default:
            break;
    }
}


void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    esp_err_t ret;

    switch (event) {
        // * Start advertising following successfull configuration of the advertising data.
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret == ESP_OK) {
                ESP_LOGI(GATTS_TAG, "%s -> Started advertising successfully! LINE %d", __func__, __LINE__);
            }
            break;

        // * Check whether the advertising was successfull.
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failure!");
            }
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;

        case ESP_GAP_BLE_PASSKEY_REQ_EVT:
            ESP_LOGI("BLE", "Passkey requested");
            esp_ble_passkey_reply(param->ble_security.auth_cmpl.bd_addr, true, 123456);
            break;

        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI("BLE", "Security request received");
            esp_ble_gap_security_rsp(param->ble_security.auth_cmpl.bd_addr, true);
            break;

        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI("BLE", "Authentication successful");
            } else {
                ESP_LOGE("BLE", "Authentication failed");
            }
            break;
        
        default:
            break;
    }
}


esp_err_t bt_init() {
    esp_err_t ret;

    // Enable security
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));

    // Initialize non-volatile storage to store non-volataile bluetooth data
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to initialize bluetooth controller", __func__);
        return ret;
    }

    esp_bt_controller_disable();
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to enable bluetooth controller", __func__);
        return ret;
    }

    // Initialize and enable bluedroid bluetooth stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to initialize bluedroid stack", __func__);
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to enable bluedroid stack", __func__);
        return ret;
    }

    // Register callback functions (functions that perform logic in response to bluetooth events)
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return ret;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return ret;
    }

    // Register gatts profile
    ret = esp_ble_gatts_app_register(MOTION_CONTROLLER_APP_ID);
    if (ret != ESP_OK){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }

    return ESP_OK;
}


