#ifndef ble_conn_h
#define ble_conn_h

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#define HID_SERVICE_UUID                    0x1812
#define HID_SERVICE_INST                    0

#define GATT_SERVICE_DECL_UUID              0x2800
#define GATT_CHAR_DECL_UUID                 0x2803
#define GATT_CHAR_DECL_SIZE                 (sizeof(uint8_t))

#define HID_INFO_UUID                       0x2A4A
#define HID_REPORT_MAP_UUID                 0x2A4B
#define HID_CONTROL_POINT_UUID              0x2A4C
#define HID_REPORT_UUID                     0x2A4D
#define HID_BOOT_MOUSE_INPUT_UUID           0x2A33

#define HID_REPORT_MAP_MAX_SIZE             512U

#define MOTION_CONTROLLER_PROFILE_NUM       1
#define MOTION_CONTROLLER_PROFILE_APP_IDX   0
#define MOTION_CONTROLLER_APP_ID            0x44


enum {
    // IDX = Index
    // CHAR = Characteristic
    // DECL = Declaration
    IDX_HID_SERVICE,

    IDX_CHAR_HID_INFO_DECL,
    IDX_CHAR_HID_INFO_VAL,

    IDX_CHAR_HID_REPORT_MAP_DECL,
    IDX_CHAR_HID_REPORT_MAP_VAL,

    IDX_CHAR_HID_CONTROL_POINT_DECL,
    IDX_CHAR_HID_CONTROL_POINT_VAL,

    IDX_CHAR_HID_REPORT_DECL,
    IDX_CHAR_HID_REPORT_VAL,

    // IDX_CHAR_HID_BOOT_MOUSE_INPUT_REPORT_DECL,
    // IDX_CHAR_HID_BOOT_MOUSE_INPUT_REPORT_VAL,

    HID_IDX_NUM,  // Length of the enum
};


extern const esp_gatts_attr_db_t hid_gatt_db[HID_IDX_NUM];

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

esp_err_t bt_init();


#endif
