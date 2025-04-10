#ifndef ble_conn_setup_h
#define ble_conn_setup_h

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"


#define GATT_CHAR_DECL_SIZE                 (sizeof(uint8_t))

#define MOTION_CONTROLLER_PROFILE_NUM       1                       // The total number of profiles in the profile tab
#define MOTION_CONTROLLER_APP_ID            0x44

// extern gatts_profile_inst motion_controller_profile_tab[MOTION_CONTROLLER_PROFILE_NUM];

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
    
    HID_IDX_NUM,  // Length of the enum
};


typedef struct {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} gatts_profile_inst;


void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
esp_err_t bt_init();


#endif
