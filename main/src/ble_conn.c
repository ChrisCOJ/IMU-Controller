#include <string.h>

#include "../include/ble_conn.h"
#include "../include/config.h"

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


#define INITIAL_HID_CONTROL_POINT_VALUE     0x00


static uint8_t hid_service_uuid[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00
};

static esp_gatt_srvc_id_t hid_service_id = {
    .is_primary = true,
    .id.inst_id = 0x00,
    .id.uuid.len = ESP_UUID_LEN_128
};

/* Characteristic uuid definitions */
static const uint8_t gatt_service_decl_uuid[2] = { GATT_SERVICE_DECL_UUID >> 8, GATT_SERVICE_DECL_UUID & 0x00FF };
static const uint8_t gatt_char_decl_uuid[2] = { GATT_CHAR_DECL_UUID >> 8, GATT_CHAR_DECL_UUID & 0x00FF };
static const uint8_t hid_info_uuid[2] = { HID_INFO_UUID >> 8, HID_INFO_UUID & 0x00FF };
static const uint8_t hid_report_map_uuid[2] = { HID_REPORT_MAP_UUID >> 8, HID_REPORT_MAP_UUID & 0x00FF };
static const uint8_t hid_control_point_uuid[2] = { HID_CONTROL_POINT_UUID >> 8, HID_CONTROL_POINT_UUID & 0x00FF };
static const uint8_t hid_report_uuid[2] = { HID_REPORT_UUID >> 8, HID_REPORT_UUID & 0x00FF };

/* Characteristic property definitions */
static const esp_gatt_char_prop_t char_prop_read = 0 | ESP_GATT_CHAR_PROP_BIT_READ;
static const esp_gatt_char_prop_t char_prop_read_write = 0 | (ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE);
static const esp_gatt_char_prop_t char_prop_write_nr = 0 | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
//static const esp_gatt_char_prop_t char_prop_write = 0 | ESP_GATT_CHAR_PROP_BIT_WRITE;

// HID Report Map for Mouse
// Documentation: https://www.usb.org/sites/default/files/documents/hid1_11.pdf (page 23)
static uint8_t hid_report_map[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Buttons)
    0x19, 0x01,        //     Usage Minimum (Button 1)
    0x29, 0x03,        //     Usage Maximum (Button 3)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x03,        //     Input (Const,Var,Abs)  ; 5 bits padding
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

static uint16_t hid_handle_table[HID_IDX_NUM];

// Init hid_report with default values. These will be changed later inside the gatts callback function.
static uint8_t hid_report[] = {
    0x00,       // Buttons pressed: 0x00 means no buttons pressed
    0x00,       // X-axis movement: 0x00 means no movement
    0x00        // Y-axis movement: 0x00 means no movement
};

static uint8_t hid_info[] = {
    0x01, 0x01,  // bcdHID: HID Class Specification release number (1.1)
    0x00,        // bCountryCode: Hardware target country (0 = Not supported)
    0x02         // Flags: 0x02 = remote wake
};

static uint8_t hid_control_point = INITIAL_HID_CONTROL_POINT_VALUE;

// Hid gatt table populated with HID attributes
const esp_gatts_attr_db_t hid_gatt_db[HID_IDX_NUM] = {
    // HID Service.
    [IDX_HID_SERVICE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)gatt_service_decl_uuid, ESP_GATT_PERM_READ, 
         sizeof(uint16_t), sizeof(hid_service_uuid), (uint8_t *)hid_service_uuid}
    },

    /* 
    * Characteristic declaration: HID info.
    * Description: Holds general information about the ble peripheral such as version, operation intended country and certain flags.
    */
    [IDX_CHAR_HID_INFO_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)gatt_char_decl_uuid, ESP_GATT_PERM_READ, 
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_read}
    },
    /* 
    * Characteristic value: HID info.
    * Value type: uint8 array
    */
    [IDX_CHAR_HID_INFO_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)hid_info_uuid, ESP_GATT_PERM_READ, 
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)hid_info}
    },

    /*
    * Characteristic declaration: HID report map.
    * Description: Holds parameters of the HID device in a logical way. 
    * Purpose: Inform the HID host of the HID device's components and possible functionality.
    */
    [IDX_CHAR_HID_REPORT_MAP_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)gatt_char_decl_uuid, ESP_GATT_PERM_READ, 
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_read}
    },
    /* 
    * Characteristic value: HID report map.
    * Value type: uint8 array
    */
    [IDX_CHAR_HID_REPORT_MAP_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)hid_report_map_uuid, ESP_GATT_PERM_READ, 
         HID_REPORT_MAP_MAX_SIZE, sizeof(hid_report_map), (uint8_t *)hid_report_map}
    },

    /*
    * Characteristic declaration: HID control point.
    * Description: Setting this to 0x00 suspends the activity of the HID device. Setting it to 0x01 resumes the activity.
    */
    [IDX_CHAR_HID_CONTROL_POINT_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)gatt_char_decl_uuid, ESP_GATT_PERM_READ, 
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_write_nr}
    },
    /*
    * Characteristic value: HID control point.
    * Value Type: uint8
    */
    [IDX_CHAR_HID_CONTROL_POINT_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)hid_control_point_uuid, ESP_GATT_PERM_WRITE, 
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&hid_control_point}
    },

    /*
    * Characteristic declaration: HID report.
    * Description: Holds the actual values read from the HID device (e.g. holds the x and y axis movement of an HID mouse)
    */
    [IDX_CHAR_HID_REPORT_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)gatt_char_decl_uuid, ESP_GATT_PERM_READ, 
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_read_write}
    },
    /*
    * Characteristic value: HID report.
    * Value type: uint8 array
    */
    [IDX_CHAR_HID_REPORT_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)hid_report_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(hid_report), sizeof(hid_report), (uint8_t *)hid_report}
    }
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,                 // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010,                 // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,                   // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hid_service_uuid),
    .p_service_uuid = hid_service_uuid,
    .flag = 0x6,
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

            ret = esp_ble_gatts_create_attr_tab(hid_gatt_db, gatts_if, HID_IDX_NUM, 1);
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
            // Fill service_uuid parameter of service_id struct
            memcpy(hid_service_id.id.uuid.uuid.uuid128, hid_service_uuid, sizeof(hid_service_uuid));

            esp_ble_gatts_create_service(gatts_if, &hid_service_id, HID_IDX_NUM);
            break;


        case ESP_GATTS_CREATE_EVT:
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
                ESP_LOGI(GATTS_TAG, "%u", *(param->add_attr_tab.handles));  // debug
                memcpy(hid_handle_table, param->add_attr_tab.handles, HID_IDX_NUM);
                esp_ble_gatts_start_service(hid_handle_table[IDX_HID_SERVICE]);
            }
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret == ESP_OK) {
                ESP_LOGI(GATTS_TAG, "%s -> Device disconnected! Restarting advertising! LINE %d", __func__, __LINE__);
            }
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
        
        default:
            break;
    }
}


esp_err_t bt_init() {
    esp_err_t ret;

    // Initialize non-volatile storage to store non-volataile bluetooth data
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    ESP_LOGI(GATTS_TAG, "1");

    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to initialize bluetooth controller", __func__);
        return ret;
    }
    ESP_LOGI(GATTS_TAG, "2");

    esp_bt_controller_disable();
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to enable bluetooth controller", __func__);
        return ret;
    }
    ESP_LOGI(GATTS_TAG, "3");

    // Initialize and enable bluedroid bluetooth stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to initialize bluedroid stack", __func__);
        return ret;
    }
    ESP_LOGI(GATTS_TAG, "4");
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to enable bluedroid stack", __func__);
        return ret;
    }
    ESP_LOGI(GATTS_TAG, "5");

    // Register callback functions (functions that perform logic in response to bluetooth events)
    esp_ble_gap_register_callback(gap_event_handler);
    ESP_LOGI(GATTS_TAG, "6");
    esp_ble_gatts_register_callback(gatts_event_handler);
    ESP_LOGI(GATTS_TAG, "7");

    // Register gatts profile
    esp_ble_gatts_app_register(MOTION_CONTROLLER_APP_ID);
    ESP_LOGI(GATTS_TAG, "8");

    return ESP_OK;
}


