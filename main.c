#include "CH58x_common.h"
#include "usbdev.h"
#include "keycodes.h"
#include <stdint.h>

int main(void) {
  SetSysClock(CLK_SOURCE_PLL_60MHz);

  PFIC_DisableIRQ(USB_IRQn);
  usbdev_init();
  PFIC_EnableIRQ(USB_IRQn);

  while (1) {
    /* reference:
       https://wiki.osdev.org/USB_Human_Interface_Devices
       https://www.usb.org/sites/default/files/hid1_11.pdf
    */
    // usb keyboard demo
#if 0
    /*
      hid boot keyboard format:
      [MOD 1B][RESV 1B][KEY1 1B][KEY2 1B]....[KEY6 1B]
     */
    while(!usb_keyboard_tx_is_ready());
    usb_keyboard_tx(KEY_B << 16);
    while(!usb_keyboard_tx_is_ready());
    usb_keyboard_tx(0x0); // release
    while(!usb_keyboard_tx_is_ready());
    usb_keyboard_tx(KEY_A << 16);
    while(!usb_keyboard_tx_is_ready());
    usb_keyboard_tx(0x0);
    while(!usb_keyboard_tx_is_ready());
    usb_keyboard_tx(KEY_D << 16);
    while(!usb_keyboard_tx_is_ready());
    usb_keyboard_tx(0x0);
#endif
    // usb mouse demo
#if 0
    /*
      hid boot mouse format:
      [BTN 1B][XMOVE 1B][Y_MOVE 1B][RESV 1B]
     */
    while(!usb_mouse_tx_is_ready());
    usb_mouse_tx(MOUSE_BTN_LEFT);
    while(!usb_mouse_tx_is_ready());
    usb_mouse_tx(0x0); // release
#endif
  }
}
