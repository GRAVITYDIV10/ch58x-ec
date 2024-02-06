#include "usbdev.h"
#include "CH58x_common.h"
#include "CH58x_usbdev.h"

const unsigned char usbdev_desc[] = {
    0x12,       // bLength
    0x01,       // bDescriptorType (Device)
    0x10, 0x01, // bcdUSB 1.10
    0x00,       // bDeviceClass Use class code info from Interface Descriptors
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocol
    0x40,       // bMaxPacketSize0 64
    0x23, 0x01, // idVendor  0x0123
    0x67, 0x45, // idProduct 0x4567
    0x00, 0x01, // bcdDevice 2.00
    0x01,       // iManufacturer (String Index)
    0x02,       // iProduct (String Index)
    0x03,       // iSerialNumber (String Index)
    0x01,       // bNumConfigurations 1

    // 18 bytes
};

/*
  HID BOOT KEYBOARD IF0 EP 0x81
  HID BOOT MOUSE IF1 EP 0x82
  HID KEYBOARD IF2 EP 0x83
  HID MOUSE IF3 EP 0x84
  HID TABLET IF4 EP 0x85
 */

const unsigned char usbdev_cfg[] = {
    0x09,       // bLength
    0x02,       // bDescriptorType (Configuration)
    0x86, 0x00, // wTotalLength 134
    0x05,       // bNumInterfaces 5
    0x01,       // bConfigurationValue
    0x00,       // iConfiguration (String Index)
    0xE0,       // bmAttributes Self Powered Remote Wakeup
    0x32,       // bMaxPower 100mA

    // 9 bytes

    // HID BOOT Keyboard
    0x09, // bLength
    0x04, // bDescriptorType (Interface)
    0x00, // bInterfaceNumber 0
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints 1
    0x03, // bInterfaceClass HID
    0x01, // bInterfaceSubClass BOOT
    0x01, // bInterfaceProtocol KEYBOARD
    0x04, // iInterface (String Index)

    // 18 bytes

    0x09,       // bLength
    0x21,       // bDescriptorType (HID)
    0x11, 0x01, // hid class
    0x00,       // country code
    0x01,       // num_descriptors
    0x22,       // type: report
    0x3f, 0x00, // length 63

    // 27 bytes

    0x07,       // bLength
    0x05,       // bDescriptorType (Endpoint)
    0x81,       // bEndpointAddress (IN/D2H)
    0x03,       // bmAttributes (Interrupt)
    0x08, 0x00, // wMaxPacketSize 8
    0x0a,       // bInterval 10 (unit depends on device speed)

    // 34 bytes

    // HID BOOT Mouse
    0x09, // bLength
    0x04, // bDescriptorType (Interface)
    0x01, // bInterfaceNumber 1
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints 1
    0x03, // bInterfaceClass HID
    0x01, // bInterfaceSubClass BOOT
    0x02, // bInterfaceProtocol MOUSE
    0x05, // iInterface (String Index)

    // 43 bytes

    0x09,       // bLength
    0x21,       // bDescriptorType (HID)
    0x01, 0x00, // hid class
    0x00,       // country code
    0x01,       // num_descriptors
    0x22,       // type: report
    0x34, 0x00, // length 52

    // 52 bytes

    0x07,       // bLength
    0x05,       // bDescriptorType (Endpoint)
    0x82,       // bEndpointAddress (IN/D2H)
    0x03,       // bmAttributes (Interrupt)
    0x04, 0x00, // wMaxPacketSize 4
    0x0a,       // bInterval 10 (unit depends on device speed)

    // 59 bytes

    // HID Keyboard
    0x09, // bLength
    0x04, // bDescriptorType (Interface)
    0x02, // bInterfaceNumber 2
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints 1
    0x03, // bInterfaceClass HID
    0x00, // bInterfaceSubClas
    0x01, // bInterfaceProtocol KEYBOARD
    0x06, // iInterface (String Index)

    // 68 bytes

    0x09,       // bLength
    0x21,       // bDescriptorType (HID)
    0x11, 0x01, // hid class
    0x00,       // country code
    0x01,       // num_descriptors
    0x22,       // type: report
    0x3f, 0x00, // length 63

    // 77 bytes

    0x07,       // bLength
    0x05,       // bDescriptorType (Endpoint)
    0x83,       // bEndpointAddress (IN/D2H)
    0x03,       // bmAttributes (Interrupt)
    0x08, 0x00, // wMaxPacketSize 8
    0x0a,       // bInterval 10 (unit depends on device speed)

    // 84 bytes

    // HID Mouse
    0x09, // bLength
    0x04, // bDescriptorType (Interface)
    0x03, // bInterfaceNumber 3
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints 1
    0x03, // bInterfaceClass HID
    0x00, // bInterfaceSubClass
    0x02, // bInterfaceProtocol MOUSE
    0x07, // iInterface (String Index)

    // 93 bytes

    0x09,       // bLength
    0x21,       // bDescriptorType (HID)
    0x01, 0x00, // hid class
    0x00,       // country code
    0x01,       // num_descriptors
    0x22,       // type: report
    0x34, 0x00, // length 52

    // 102 bytes

    0x07,       // bLength
    0x05,       // bDescriptorType (Endpoint)
    0x84,       // bEndpointAddress (IN/D2H)
    0x03,       // bmAttributes (Interrupt)
    0x04, 0x00, // wMaxPacketSize 4
    0x0a,       // bInterval 10 (unit depends on device speed)

    // 109 bytes

    // HID tablet
    0x09, // bLength
    0x04, // bDescriptorType (Interface)
    0x04, // bInterfaceNumber 4
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints 1
    0x03, // bInterfaceClass HID
    0x00, // bInterfaceSubClass
    0x00, // bInterfaceProtocol CUSTOM
    0x08, // iInterface (String Index)

    // 118 bytes

    0x09,       // bLength
    0x21,       // bDescriptorType (HID)
    0x01, 0x00, // hid class
    0x00,       // country code
    0x01,       // num_descriptors
    0x22,       // type: report
    0x4a, 0x00, // length 74

    // 127 bytes

    0x07,       // bLength
    0x05,       // bDescriptorType (Endpoint)
    0x85,       // bEndpointAddress (IN/D2H)
    0x03,       // bmAttributes (Interrupt)
    0x06, 0x00, // wMaxPacketSize 6
    0x0A,       // bInterval 10 (unit depends on device speed)

    // 134 bytes

};

const uint8_t usb_dev_lang_desc[] = { 0x04, 0x03, 0x09, 0x04 };
const uint8_t usb_dev_str_manuinfo[] = { 0x08, 0x03, 'w', 0, 'c', 0, 'h', 0, };
const uint8_t usb_dev_str_prodinfo[] = { 0x08, 0x03, '5', 0, '8', 0, 'x', 0, };
const uint8_t usb_dev_str_serialnuminfo[] = { 0x10, 0x03, '0', 0, '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, };
const uint8_t usb_dev_str_hid_boot_keyboard[] = {
    0x22, 0x03, 'h', 0, 'i', 0, 'd', 0, ' ', 0, 'b', 0, 'o', 0, 'o', 0, 't', 0,
    'k',  0,    'e', 0, 'y', 0, 'b', 0, 'o', 0, 'a', 0, 'r', 0, 'd', 0,
};

const uint8_t usb_dev_str_hid_boot_mouse[] = {
    0x1c, 0x03, 'h', 0, 'i', 0, 'd', 0, ' ', 0, 'b', 0, 'o', 0,
    'o',  0,    't', 0, 'm', 0, 'o', 0, 'u', 0, 's', 0, 'e', 0,
};

const uint8_t usb_dev_str_hid_keyboard[] = {
    0x1a, 0x03, 'h', 0, 'i', 0, 'd', 0, ' ', 0, 'k', 0, 'e', 0, 'y', 0, 'b', 0,
    'o', 0, 'a', 0, 'r', 0, 'd', 0,
};

const uint8_t usb_dev_str_hid_mouse[] = {
    0x14, 0x03, 'h', 0, 'i', 0, 'd', 0, ' ', 0,
    'm',  0,    'o', 0, 'u', 0, 's', 0, 'e', 0,
};

const uint8_t usb_dev_str_hid_tablet[] = {
  0x16, 0x03, 'h', 0, 'i', 0, 'd', 0,
  ' ',  0,    't', 0, 'a', 0, 'b', 0,
  'l',  0,    'e', 0, 't', 0};

const uint8_t hid_boot_keyboard_desc[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
  0x09, 0x06,        // Usage (Keyboard)
  0xA1, 0x01,        // Collection (Application)
  0x75, 0x01,        //   Report Size (1)
  0x95, 0x08,        //   Report Count (8)
  0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
  0x19, 0xE0,        //   Usage Minimum (0xE0)
  0x29, 0xE7,        //   Usage Maximum (0xE7)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x01,        //   Logical Maximum (1)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x95, 0x01,        //   Report Count (1)
  0x75, 0x08,        //   Report Size (8)
  0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x95, 0x05,        //   Report Count (5)
  0x75, 0x01,        //   Report Size (1)
  0x05, 0x08,        //   Usage Page (LEDs)
  0x19, 0x01,        //   Usage Minimum (Num Lock)
  0x29, 0x05,        //   Usage Maximum (Kana)
  0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
  0x95, 0x01,        //   Report Count (1)
  0x75, 0x03,        //   Report Size (3)
  0x91, 0x01,        //   Output (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
  0x95, 0x06,        //   Report Count (6)
  0x75, 0x08,        //   Report Size (8)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0xFF,        //   Logical Maximum (-1)
  0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
  0x19, 0x00,        //   Usage Minimum (0x00)
  0x29, 0xFF,        //   Usage Maximum (0xFF)
  0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,              // End Collection
  // 63 bytes
};
const uint8_t hid_boot_mouse_desc[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x05, 0x09,        //     Usage Page (Button)
  0x19, 0x01,        //     Usage Minimum (0x01)
  0x29, 0x05,        //     Usage Maximum (0x05)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x95, 0x05,        //     Report Count (5)
  0x75, 0x01,        //     Report Size (1)
  0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x95, 0x01,        //     Report Count (1)
  0x75, 0x03,        //     Report Size (3)
  0x81, 0x01,        //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
  0x09, 0x30,        //     Usage (X)
  0x09, 0x31,        //     Usage (Y)
  0x09, 0x38,        //     Usage (Wheel)
  0x15, 0x81,        //     Logical Minimum (-127)
  0x25, 0x7F,        //     Logical Maximum (127)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x03,        //     Report Count (3)
  0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,              //   End Collection
  0xC0,              // End Collection
  // 52 bytes
};

const uint8_t hid_tablet_desc[] = {
  // reference: qemu/hw/usb/dev-hid.c
  0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x05, 0x09,        //     Usage Page (Button)
  0x19, 0x01,        //     Usage Minimum (0x01)
  0x29, 0x03,        //     Usage Maximum (0x03)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x95, 0x03,        //     Report Count (3)
  0x75, 0x01,        //     Report Size (1)
  0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x95, 0x01,        //     Report Count (1)
  0x75, 0x05,        //     Report Size (5)
  0x81, 0x01,        //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
  0x09, 0x30,        //     Usage (X)
  0x09, 0x31,        //     Usage (Y)
  0x15, 0x00,        //     Logical Minimum (0)
  0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
  0x35, 0x00,        //     Physical Minimum (0)
  0x46, 0xFF, 0x7F,  //     Physical Maximum (32767)
  0x75, 0x10,        //     Report Size (16)
  0x95, 0x02,        //     Report Count (2)
  0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
  0x09, 0x38,        //     Usage (Wheel)
  0x15, 0x81,        //     Logical Minimum (-127)
  0x25, 0x7F,        //     Logical Maximum (127)
  0x35, 0x00,        //     Physical Minimum (0)
  0x45, 0x00,        //     Physical Maximum (0)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,              //   End Collection
  0xC0,              // End Collection
  // 74 bytes
};


#define DevEP0SIZE 0x40

// ep0(64)+ep4_out(64)+ep4_in(64)
__attribute__((aligned(4))) uint8_t EP0_Databuf[64 + 64 + 64];
// ep1_out(64)+ep1_in(64)
__attribute__((aligned(4))) uint8_t EP1_Databuf[64 + 64];
__attribute__((aligned(4))) uint8_t EP2_Databuf[64 + 64];
__attribute__((aligned(4))) uint8_t EP3_Databuf[64 + 64];
__attribute__((aligned(4))) uint8_t EP5_Databuf[64 + 64];

uint8_t *pEP5_RAM_Addr;
#define pEP5_OUT_DataBuf      (pEP5_RAM_Addr)
#define pEP5_IN_DataBuf       (pEP5_RAM_Addr + 64)
void DevEP5_IN_Deal(uint8_t l)
{
  R8_UEP5_T_LEN = l;
  R8_UEP5_CTRL = (R8_UEP5_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
}
#define EP5_GetINSta()    (R8_UEP5_CTRL & UEP_T_RES_NAK)

int usb_hid_boot_keyboard_tx_is_ready(void) { return (EP1_GetINSta()); }

void usb_hid_boot_keyboard_tx(uint64_t code) {
  if (usb_hid_boot_keyboard_tx_is_ready()) {
    memcpy(pEP1_IN_DataBuf, &code, 8);
    DevEP1_IN_Deal(8);
  }
}

int usb_hid_boot_mouse_tx_is_ready(void) { return (EP2_GetINSta()); }

void usb_hid_boot_mouse_tx(uint32_t code) {
  if (usb_hid_boot_mouse_tx_is_ready()) {
    memcpy(pEP2_IN_DataBuf, &code, 4);
    DevEP2_IN_Deal(4);
  }
}

int usb_hid_keyboard_tx_is_ready(void) { return (EP3_GetINSta()); }

void usb_hid_keyboard_tx(uint64_t code) {
  if (usb_hid_keyboard_tx_is_ready()) {
    memcpy(pEP3_IN_DataBuf, &code, 8);
    DevEP3_IN_Deal(8);
  }
}

int usb_hid_mouse_tx_is_ready(void) { return (EP4_GetINSta()); }

void usb_hid_mouse_tx(uint32_t code) {
  if (usb_hid_mouse_tx_is_ready()) {
    memcpy(pEP4_IN_DataBuf, &code, 4);
    DevEP4_IN_Deal(4);
  }
}

int usb_hid_tablet_tx_is_ready(void) { return (EP5_GetINSta()); }

void usb_hid_tablet_tx(uint64_t code) {
  if (usb_hid_tablet_tx_is_ready()) {
    memcpy(pEP5_IN_DataBuf, &code, 6);
    DevEP5_IN_Deal(6);
  }
}

#define DevEP0SIZE 0x40

uint8_t DevConfig;
uint8_t SetupReqCode;
uint16_t SetupReqLen;
const uint8_t *pDescr;

uint8_t usb_hiddev_idle_val[8];
uint8_t usb_hiddev_protocol[8];

__HIGH_CODE
void USB_DevTransProcess(void) {
  uint8_t len, chtype;
  uint8_t intflag, errflag = 0;

  intflag = R8_USB_INT_FG;
  if (intflag & RB_UIF_TRANSFER) {
    if ((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN) // 非空闲
    {
      switch (R8_USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
      // 分析操作令牌和端点号
      {
      case UIS_TOKEN_IN: {
        switch (SetupReqCode) {
        case USB_GET_DESCRIPTOR:
          len = SetupReqLen >= DevEP0SIZE ? DevEP0SIZE
                                          : SetupReqLen; // 本次传输长度
          memcpy(pEP0_DataBuf, pDescr, len); /* 加载上传数据 */
          SetupReqLen -= len;
          pDescr += len;
          R8_UEP0_T_LEN = len;
          R8_UEP0_CTRL ^= RB_UEP_T_TOG; // 翻转
          break;
        case USB_SET_ADDRESS:
          R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | SetupReqLen;
          R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
          break;
        default:
          R8_UEP0_T_LEN =
              0; // 状态阶段完成中断或者是强制上传0长度数据包结束控制传输
          R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
          break;
        }
      } break;
      case UIS_TOKEN_OUT:
        len = R8_USB_RX_LEN;
        break;
      case UIS_TOKEN_OUT | 1: {
        if (R8_USB_INT_ST & RB_UIS_TOG_OK) { // 不同步的数据包将丢弃
          R8_UEP1_CTRL ^= RB_UEP_R_TOG;
          len = R8_USB_RX_LEN;
          //DevEP1_OUT_Deal(len);
        }
      } break;
      case UIS_TOKEN_IN | 1:
        R8_UEP1_CTRL ^= RB_UEP_T_TOG;
        R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
      case UIS_TOKEN_OUT | 2: {
        if (R8_USB_INT_ST & RB_UIS_TOG_OK) { // 不同步的数据包将丢弃
          R8_UEP2_CTRL ^= RB_UEP_R_TOG;
          len = R8_USB_RX_LEN;
          //DevEP2_OUT_Deal(len);
        }
      } break;
      case UIS_TOKEN_IN | 2:
        R8_UEP2_CTRL ^= RB_UEP_T_TOG;
        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
      case UIS_TOKEN_OUT | 3: {
        if (R8_USB_INT_ST & RB_UIS_TOG_OK) { // 不同步的数据包将丢弃
          R8_UEP3_CTRL ^= RB_UEP_R_TOG;
          len = R8_USB_RX_LEN;
          //DevEP3_OUT_Deal(len);
        }
      } break;
      case UIS_TOKEN_IN | 3:
        R8_UEP3_CTRL ^= RB_UEP_T_TOG;
        R8_UEP3_CTRL = (R8_UEP3_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
      case UIS_TOKEN_OUT | 4: {
        if (R8_USB_INT_ST & RB_UIS_TOG_OK) { // 不同步的数据包将丢弃
          R8_UEP4_CTRL ^= RB_UEP_R_TOG;
          len = R8_USB_RX_LEN;
          //DevEP4_OUT_Deal(len);
        }
      } break;
      case UIS_TOKEN_IN | 4:
        R8_UEP4_CTRL ^= RB_UEP_T_TOG;
        R8_UEP4_CTRL = (R8_UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
      case UIS_TOKEN_OUT | 5: {
        if (R8_USB_INT_ST & RB_UIS_TOG_OK) { // 不同步的数据包将丢弃
          R8_UEP5_CTRL ^= RB_UEP_R_TOG;
          len = R8_USB_RX_LEN;
          //DevEP5_OUT_Deal(len);
        }
      } break;
      case UIS_TOKEN_IN | 5:
        R8_UEP5_CTRL ^= RB_UEP_T_TOG;
        R8_UEP5_CTRL = (R8_UEP5_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
      default:
        break;
      }
      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }

    if (R8_USB_INT_ST & RB_UIS_SETUP_ACT) { // Setup包处理
      R8_UEP0_CTRL =
          RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
      SetupReqLen = pSetupReqPak->wLength;
      SetupReqCode = pSetupReqPak->bRequest;
      chtype = pSetupReqPak->bRequestType;

      len = 0;
      errflag = 0;
      if ((pSetupReqPak->bRequestType & USB_REQ_TYP_MASK) !=
          USB_REQ_TYP_STANDARD) {
        if (pSetupReqPak->bRequestType & 0x20) { // HID request
          switch (SetupReqCode) {
          case DEF_USB_SET_IDLE:
            usb_hiddev_idle_val[pSetupReqPak->wIndex] =
                (uint8_t)(pSetupReqPak->wValue >> 8);
	    break;
          case DEF_USB_SET_REPORT:
            break;
          case DEF_USB_SET_PROTOCOL:
            usb_hiddev_protocol[pSetupReqPak->wIndex] =
	      (uint8_t)(pSetupReqPak->wValue);
            break;
          case DEF_USB_GET_IDLE:
            EP0_Databuf[0] = usb_hiddev_idle_val[pSetupReqPak->wIndex];
            len = 1;
            break;
          case DEF_USB_GET_PROTOCOL:
            EP0_Databuf[0] = usb_hiddev_protocol[pSetupReqPak->wIndex];
            len = 1;
            break;
          default:
            errflag = 0xff;
	    break;
	  }
	}
        //errflag = 0xFF; /* 非标准请求 */
      } else { /* 标准请求 */
        switch (SetupReqCode) {
        case USB_GET_DESCRIPTOR: {
          switch (((pSetupReqPak->wValue) >> 8)) {
          case USB_DESCR_TYP_DEVICE: {
            pDescr = usbdev_desc;
            len = usbdev_desc[0];
          } break;
          case USB_DESCR_TYP_CONFIG: {
            pDescr = usbdev_cfg;
            len = usbdev_cfg[2];
          } break;
          case USB_DESCR_TYP_STRING: {
            switch ((pSetupReqPak->wValue) & 0xff) {
            case 0:
              pDescr = usb_dev_lang_desc;
              len = usb_dev_lang_desc[0];
              break;
	    case 1:
              pDescr = usb_dev_str_manuinfo;
              len = usb_dev_str_manuinfo[0];
              break;
	    case 2:
              pDescr = usb_dev_str_prodinfo;
              len = usb_dev_str_prodinfo[0];
              break;
	    case 3:
              pDescr = usb_dev_str_serialnuminfo;
              len = usb_dev_str_serialnuminfo[0];
              break;
	    case 4:
              pDescr = usb_dev_str_hid_boot_keyboard;
              len = usb_dev_str_hid_boot_keyboard[0];
              break;
	    case 5:
              pDescr = usb_dev_str_hid_boot_mouse;
              len = usb_dev_str_hid_boot_mouse[0];
              break;
	    case 6:
              pDescr = usb_dev_str_hid_keyboard;
              len = usb_dev_str_hid_keyboard[0];
              break;
	    case 7:
              pDescr = usb_dev_str_hid_mouse;
              len = usb_dev_str_hid_mouse[0];
              break;
	    case 8:
              pDescr = usb_dev_str_hid_tablet;
              len = usb_dev_str_hid_tablet[0];
	      break;
            default:
              errflag = 0xFF; // 不支持的字符串描述符
              break;
            }
          } break;
          case USB_DESCR_TYP_HID: {
	    switch((pSetupReqPak->wIndex) & 0xff) {
            case 0: // hid boot keyboard
              pDescr = (uint8_t *)(&usbdev_cfg[18]);
	      len = 9;
	      break;
            case 1: // hid boot mouse
	      pDescr = (uint8_t *)(&usbdev_cfg[43]);
	      len = 9;
	      break;
            case 2: // hid keyboard
	      pDescr = (uint8_t *)(&usbdev_cfg[68]);
	      len = 9;
              break;
            case 3: // hid mouse
	      pDescr = (uint8_t *)(&usbdev_cfg[93]);
	      len = 9;
              break;
	    case 4: // hid tablet
	      pDescr = (uint8_t *)(&usbdev_cfg[118]);
	      len = 9;
              break;
            default:
	      errflag = 0xff;
	      break;
	    }
          } break;
          case USB_DESCR_TYP_REPORT: {
            switch ((pSetupReqPak->wIndex) & 0xff) {
            case 0:
              pDescr = hid_boot_keyboard_desc;
	      len = sizeof(hid_boot_keyboard_desc);
	      break;
            case 1:
              pDescr = hid_boot_mouse_desc;
	      len = sizeof(hid_boot_mouse_desc);
	      break;
            case 2:
	      pDescr = hid_boot_keyboard_desc;
	      len = sizeof(hid_boot_keyboard_desc);
	      break;
            case 3:
	      pDescr = hid_boot_mouse_desc;
	      len = sizeof(hid_boot_mouse_desc);
              break;
	    case 4:
	      pDescr = hid_tablet_desc;
	      len = sizeof(hid_tablet_desc);
	      break;
            default:
	      len = 0xff;
	      break;
	    }
	  } break;
          default:
            errflag = 0xff;
            break;
          }
          if (SetupReqLen > len)
            SetupReqLen = len; //实际需上传总长度
          len = (SetupReqLen >= DevEP0SIZE) ? DevEP0SIZE : SetupReqLen;
          memcpy(pEP0_DataBuf, pDescr, len);
          pDescr += len;
        } break;
        case USB_SET_ADDRESS:
          SetupReqLen = (pSetupReqPak->wValue) & 0xff;
          break;
        case USB_GET_CONFIGURATION:
          pEP0_DataBuf[0] = DevConfig;
          if (SetupReqLen > 1) {
            SetupReqLen = 1;
	  }
          break;
        case USB_SET_CONFIGURATION:
          DevConfig = (pSetupReqPak->wValue) & 0xff;
          break;
        case USB_CLEAR_FEATURE: {
          if ((pSetupReqPak->bRequestType & USB_REQ_RECIP_MASK) ==
              USB_REQ_RECIP_ENDP) { // 端点
            switch ((pSetupReqPak->wIndex) & 0xff) {
	    case 0x85:
              R8_UEP5_CTRL = (R8_UEP5_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) |
		UEP_T_RES_NAK;
              break;
            case 0x05:
              R8_UEP5_CTRL = (R8_UEP5_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) |
		UEP_R_RES_ACK;
              break;
	    case 0x84:
              R8_UEP4_CTRL = (R8_UEP4_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) |
		UEP_T_RES_NAK;
              break;
            case 0x04:
              R8_UEP4_CTRL = (R8_UEP4_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) |
		UEP_R_RES_ACK;
              break;
	    case 0x83:
              R8_UEP3_CTRL = (R8_UEP3_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) |
                             UEP_T_RES_NAK;
              break;
            case 0x03:
              R8_UEP3_CTRL = (R8_UEP3_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) |
                             UEP_R_RES_ACK;
              break;
            case 0x82:
              R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) |
                             UEP_T_RES_NAK;
              break;
            case 0x02:
              R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) |
                             UEP_R_RES_ACK;
              break;
            case 0x81:
              R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) |
                             UEP_T_RES_NAK;
              break;
            case 0x01:
              R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) |
                             UEP_R_RES_ACK;
              break;
            default:
              errflag = 0xFF; // 不支持的端点
              break;
            }
          } else {
            errflag = 0xFF;
	  }
        } break;
        case USB_GET_INTERFACE:
          pEP0_DataBuf[0] = 0x00;
          if (SetupReqLen > 1) {
            SetupReqLen = 1;
	  }
          break;
        case USB_GET_STATUS:
          pEP0_DataBuf[0] = 0x00;
          pEP0_DataBuf[1] = 0x00;
          if (SetupReqLen > 2) {
            SetupReqLen = 2;
	  }
          break;
        default:
          errflag = 0xff;
          break;
        }
      }
      if (errflag == 0xff) { // 错误或不支持
        // SetupReqCode = 0xFF;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL |
                       UEP_T_RES_STALL; // STALL
      } else {
        if (chtype & 0x80) { // 上传
          len = (SetupReqLen > DevEP0SIZE) ? DevEP0SIZE : SetupReqLen;
          SetupReqLen -= len;
        } else {
          len = 0; // 下传
	}
        R8_UEP0_T_LEN = len;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK |
                       UEP_T_RES_ACK; // 默认数据包是DATA1
      }
      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
  } else if (intflag & RB_UIF_BUS_RST) {
    R8_USB_DEV_AD = 0;
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP5_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_USB_INT_FG = RB_UIF_BUS_RST;
  } else if (intflag & RB_UIF_SUSPEND) {
    if (R8_USB_MIS_ST & RB_UMS_SUSPEND) {  // 挂起
      ;
    } else {  // 唤醒
      ;
    }
    R8_USB_INT_FG = RB_UIF_SUSPEND;
  } else {
    R8_USB_INT_FG = intflag;
  }
}

void usbdev_init(void) {
  R8_USB_CTRL = 0x00;
  R8_UEP4_1_MOD = RB_UEP1_RX_EN | RB_UEP1_TX_EN | RB_UEP4_RX_EN | RB_UEP4_TX_EN;
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN;
  R8_UEP567_MOD = RB_UEP5_RX_EN | RB_UEP5_TX_EN;

  pEP0_RAM_Addr = EP0_Databuf;
  pEP1_RAM_Addr = EP1_Databuf;
  pEP2_RAM_Addr = EP2_Databuf;
  pEP3_RAM_Addr = EP3_Databuf;
  pEP5_RAM_Addr = EP5_Databuf;

  R16_UEP0_DMA = (uint16_t)(uint32_t)pEP0_RAM_Addr;
  R16_UEP1_DMA = (uint16_t)(uint32_t)pEP1_RAM_Addr;
  R16_UEP2_DMA = (uint16_t)(uint32_t)pEP2_RAM_Addr;
  R16_UEP3_DMA = (uint16_t)(uint32_t)pEP3_RAM_Addr;
  R16_UEP5_DMA = (uint16_t)(uint32_t)pEP5_RAM_Addr;

  R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP5_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  R8_USB_DEV_AD = 0x00;
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
  R16_PIN_ANALOG_IE |= RB_PIN_USB_IE | RB_PIN_USB_DP_PU;
  R8_USB_INT_FG = 0xFF;
  R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
}

__INTERRUPT
__HIGH_CODE
void USB_IRQHandler(void) { USB_DevTransProcess(); }
