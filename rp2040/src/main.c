#include <usb-serial.h>
#include <sensors.h>

int main (void) {
    usb_serial_init ();
    sensors_init();

    char buf[200];
    
    sensor_data sd;

    while (1) {
        if (sensors_update()) {
            sensors_read(&sd);
            sprintf(buf, "TIME: %ld R: %f P: %f Y: %f uart_pos: %ld usb_pos %ld vol: %f cur: %f pow: %f temp: %f lcdata: %ld %ld %ld %ld %ld %ld\n",
                get_time_ms() / 1000,
                sd.roll, sd.pitch, sd.yaw,
                usb_com_uart_buff_index(), usb_com_usb_buff_index(),
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