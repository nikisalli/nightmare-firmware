#include <usb-serial.h>
#include <gy953.h>

gy953 mag = {
    .cs_pin = 17,
    .int_pin = 15,
    .sck_pin = 18,
    .mosi_pin = 19,
    .miso_pin = 16,
};

int main (void) {
    usb_serial_init ();
    gy953_init(&mag);

    int data[3];
    char buf[100];

    uint32_t val = 0;
    while (1) {
        val++;
        usb_serial_update ();
        if(val % 2000 == 0){
            if (gy953_update(&mag, 0)) {
                gy953_getRPY(&mag, data);
                sprintf(buf, "R: %d P: %d Y: %d\n", data[0], data[1], data[2]);
                usb_com_print(buf);
            }
        }
    }
}