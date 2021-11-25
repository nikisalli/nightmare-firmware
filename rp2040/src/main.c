#include <usb-serial.h>
#include <sensors.h>

int main (void) {
    usb_serial_init();
    sensors_init();

    char buf[200];
    
    sensor_data sd;

    while (1) {
        usb_serial_handle();
        if (sensors_update()) {
            sensors_read(&sd);
            sprintf(buf, "TIME: %ld R: %.2f P: %.2f Y: %.2f V: %.2f A: %.2f W: %.2f C°: %.1f lc: %ld %ld %ld %ld %ld %ld\n",
                get_time_ms() / 1000,
                sd.roll, sd.pitch, sd.yaw,
                sd.battery_voltage, sd.battery_current, sd.battery_power, sd.rp2040_temperature, 
                sd.load_cells_raw[0], sd.load_cells_raw[1], sd.load_cells_raw[2], sd.load_cells_raw[3], sd.load_cells_raw[4], sd.load_cells_raw[5]);
            // sprintf(buf, "TIME: %ld R: %f P: %f Y: %f uart_pos: %ld usb_pos %ld lcdata: %ld %ld %ld %ld %ld %ld\n",
            //     get_time_ms() / 1000,
            //     sd.roll, sd.pitch, sd.yaw,
            //     usb_com_uart_buff_index(), usb_com_usb_buff_index(),
            //     sd.load_cells_raw[0], sd.load_cells_raw[1], sd.load_cells_raw[2], sd.load_cells_raw[3], sd.load_cells_raw[4], sd.load_cells_raw[5]);
            usb_com_print(buf);
        }
    }
}