#include <usb-serial.h>
#include <sensors.h>

const uint8_t header[] = {0x55, 0x55, 0x55, 0x55};

uint8_t checksum(uint8_t *data, uint32_t len) {
    uint8_t sum = 0;
    for (uint32_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

int main (void) {
    usb_serial_init();
    sensors_init();

    char buf[200];
    
    sensor_data sd;

    while (1) {
        // usb_serial_handle();
        
        if (sensors_update()) {
            sensors_read(&sd);
            usb_com_write((uint8_t *)header, sizeof(header));
            usb_com_write((uint8_t *)&sd, sizeof(sd));
            usb_com_write_char(checksum((uint8_t *)&sd, sizeof(sd)));

            // sprintf(buf, "TIME: %ld R: %.2f P: %.2f Y: %.2f V: %.2f A: %.2f W: %.2f C°: %.1f lc: %ld %ld %ld %ld %ld %ld\n",
            //     get_time_ms() / 1000,
            //     sd.roll, sd.pitch, sd.yaw,
            //     sd.battery_voltage, sd.battery_current, sd.battery_power, sd.rp2040_temperature, 
            //     sd.load_cells_raw[0], sd.load_cells_raw[1], sd.load_cells_raw[2], sd.load_cells_raw[3], sd.load_cells_raw[4], sd.load_cells_raw[5]);
            // sprintf(buf, "TIME: %ld R: %f P: %f Y: %f uart_pos: %ld usb_pos %ld lcdata: %ld %ld %ld %ld %ld %ld\n",
            //     get_time_ms() / 1000,
            //     sd.roll, sd.pitch, sd.yaw,
            //     usb_com_uart_buff_index(), usb_com_usb_buff_index(),
            //     sd.load_cells_raw[0], sd.load_cells_raw[1], sd.load_cells_raw[2], sd.load_cells_raw[3], sd.load_cells_raw[4], sd.load_cells_raw[5]);
            // usb_com_print("TIME: 53 R: -0.28 P: 2.29 Y: -86.29 V: 8.35 A: 0.26 W: 2.15 C°: 36.0 lc: 41094 -70954 105278 108444 88376 -108466\n");
        }
        
        /*
        if (get_time_ms() % 20 == 0) {
            usb_com_print("alive!\0");
        }
        */
    }
}