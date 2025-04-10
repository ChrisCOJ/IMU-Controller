#ifndef mc_hid_defs_h
#define mc_hid_defs_h

#define HID_SERVICE_INST                    0
#define HID_PROFILE_APP_IDX                 0
#define HID_REPORT_LEN                      3                       // size of report = 3 bytes
#define HID_REPORT_MAP_MAX_SIZE             512U


const uint16_t HID_INFO_UUID = 0x2A4A;
const uint16_t HID_REPORT_MAP_UUID = 0x2A4B;
const uint16_t HID_CONTROL_POINT_UUID = 0x2A4C;
const uint16_t HID_REPORT_UUID = 0x2A4D;


uint8_t HID_SERVICE_UUID[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00
};


// Init hid_report with default values. These will be changed later inside the gatts callback function.
uint8_t hid_report[3] = {
    0x00,       // Buttons pressed: 0x00 means no buttons pressed
    0x00,       // X-axis movement: 0x00 means no movement
    0x00        // Y-axis movement: 0x00 means no movement
};

uint8_t hid_info[4] = {
    0x01, 0x01,  // bcdHID: HID Class Specification release number (1.1)
    0x00,        // bCountryCode: Hardware target country (0 = Not supported)
    0x02         // Flags: 0x02 = remote wake
};

uint8_t hid_control_point = 0;  // Initialize control point attr in non-suspended mode. 0 = non-suspended, 1 = suspended


// HID Report Map for Mouse
// Documentation: https://www.usb.org/sites/default/files/documents/hid1_11.pdf (page 23)
uint8_t mc_hid_report_map[] = {
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

#endif