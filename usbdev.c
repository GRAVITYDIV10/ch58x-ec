#include "usbdev.h"
#include "CH58x_common.h"

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
    0x00,       // iManufacturer (String Index)
    0x00,       // iProduct (String Index)
    0x00,       // iSerialNumber (String Index)
    0x01,       // bNumConfigurations 1

    // 18 bytes
};

/*
  HID KEYBOARD IF0 EP 0x81
  HID MOUSE IF1 EP 0x82
 */

const unsigned char usbdev_cfg[] = {
    0x09,       // bLength
    0x02,       // bDescriptorType (Configuration)
    0x29, 0x00, // wTotalLength 41
    0x02,       // bNumInterfaces 2
    0x01,       // bConfigurationValue
    0x00,       // iConfiguration (String Index)
    0xE0,       // bmAttributes Self Powered Remote Wakeup
    0x32,       // bMaxPower 100mA

    // HID BOOT Keyboard
    0x09, // bLength
    0x04, // bDescriptorType (Interface)
    0x00, // bInterfaceNumber 0
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints 1
    0x03, // bInterfaceClass HID
    0x01, // bInterfaceSubClass BOOT
    0x01, // bInterfaceProtocol KEYBOARD
    0x00, // iInterface (String Index)

    0x07,       // bLength
    0x05,       // bDescriptorType (Endpoint)
    0x81,       // bEndpointAddress (IN/D2H)
    0x03,       // bmAttributes (Interrupt)
    0x08, 0x00, // wMaxPacketSize 8
    0x01,       // bInterval 1 (unit depends on device speed)

    // HID BOOT Mouse
    0x09, // bLength
    0x04, // bDescriptorType (Interface)
    0x01, // bInterfaceNumber 1
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints 1
    0x03, // bInterfaceClass HID
    0x01, // bInterfaceSubClass BOOT
    0x02, // bInterfaceProtocol MOUSE
    0x00, // iInterface (String Index)

    0x07,       // bLength
    0x05,       // bDescriptorType (Endpoint)
    0x82,       // bEndpointAddress (IN/D2H)
    0x03,       // bmAttributes (Interrupt)
    0x04, 0x00, // wMaxPacketSize 4
    0x01,       // bInterval 1 (unit depends on device speed)

    // 41 bytes
};

#define DevEP0SIZE 0x40

// ep0(64)+ep4_out(64)+ep4_in(64)
__attribute__((aligned(4))) uint8_t EP0_Databuf[64 + 64 + 64];
// ep1_out(64)+ep1_in(64)
__attribute__((aligned(4))) uint8_t EP1_Databuf[64 + 64];
__attribute__((aligned(4))) uint8_t EP2_Databuf[64 + 64];

int usb_keyboard_tx_is_ready(void) { return (EP1_GetINSta()); }

void usb_keyboard_tx(uint64_t code) {
  if (usb_keyboard_tx_is_ready()) {
    memcpy(pEP1_IN_DataBuf, &code, 8);
    DevEP1_IN_Deal(8);
  }
}

int usb_mouse_tx_is_ready(void) { return (EP2_GetINSta()); }

void usb_mouse_tx(uint32_t code) {
  if (usb_mouse_tx_is_ready()) {
    memcpy(pEP2_IN_DataBuf, &code, 4);
    DevEP2_IN_Deal(4);
  }
}

#define DevEP0SIZE 0x40

uint8_t DevConfig;
uint8_t SetupReqCode;
uint16_t SetupReqLen;
const uint8_t *pDescr;

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
        errflag = 0xFF; /* 非标准请求 */
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
            default:
              errflag = 0xFF; // 不支持的字符串描述符
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
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN;

  pEP0_RAM_Addr = EP0_Databuf;
  pEP1_RAM_Addr = EP1_Databuf;
  pEP2_RAM_Addr = EP2_Databuf;

  R16_UEP0_DMA = (uint16_t)(uint32_t)pEP0_RAM_Addr;
  R16_UEP1_DMA = (uint16_t)(uint32_t)pEP1_RAM_Addr;
  R16_UEP2_DMA = (uint16_t)(uint32_t)pEP2_RAM_Addr;

  R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

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
