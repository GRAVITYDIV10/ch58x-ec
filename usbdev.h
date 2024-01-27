

#ifndef _USBDEV_H_
#define _USBDEV_H_

#include <stdint.h>

void usbdev_init(void);

int usb_keyboard_tx_is_ready(void);
void usb_keyboard_tx(uint64_t code);

int usb_mouse_tx_is_ready(void);
void usb_mouse_tx(uint32_t code);

#endif
