#ifndef ble_conn_setup_h
#define ble_conn_setup_h

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"


#define GATT_CHAR_DECL_SIZE                 (sizeof(uint8_t))

#define MOTION_CONTROLLER_PROFILE_NUM       1                       // The total number of profiles in the profile tab
#define MOTION_CONTROLLER_APP_ID            0x44

#define HID_REPORT_NOTIFY_ENABLE            1

extern uint8_t hid_report[3];
extern uint8_t hid_info[4];
extern uint8_t hid_control_point;
extern uint8_t mc_hid_report_map[52];
extern uint8_t report_reference_descriptor[2];
extern uint8_t HID_SERVICE_UUID[16];

enum {
    /* 
    IDX = Index
    CHAR = Characteristic
    DECL = Declaration
    */
    IDX_HID_SERVICE,
    
    IDX_CHAR_HID_INFO_DECL,
    IDX_CHAR_HID_INFO_VAL,
    
    IDX_CHAR_HID_REPORT_MAP_DECL,
    IDX_CHAR_HID_REPORT_MAP_VAL,
    
    IDX_CHAR_HID_CONTROL_POINT_DECL,
    IDX_CHAR_HID_CONTROL_POINT_VAL,
    
    IDX_CHAR_HID_REPORT_DECL,
    IDX_CHAR_HID_REPORT_VAL,
    IDX_REPORT_CCCD_DESC,  // Client Characteristic Configuration Descriptor
    IDX_REPORT_REFERENCE_DESC,
    
    HID_IDX_NUM,  // Length of the enum
};

extern uint16_t hid_handle_table[HID_IDX_NUM];  // Used to later store the handle of each gatt attribute

extern bool report_enabled_notifications;

typedef struct {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
} gatts_profile_inst;

extern gatts_profile_inst motion_controller_profile_tab[MOTION_CONTROLLER_PROFILE_NUM];

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
esp_err_t bt_init();


#endif
