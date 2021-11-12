#ifndef USB_UART_H
#define USB_UART_H

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>

void usb_serial_init(void);
void usb_serial_update(void);
uint8_t usb_com_read();
void usb_com_write(uint8_t* buf, uint32_t len);
void usb_com_print(char* buf);

#endif