#include <sensors.h>
#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>

#define LED_PIN 25
#define LED1_PIN 24

void core1_entry(void) {
	tusb_init();
	while (1) {
		tud_task();
	}
}

int main (void) {
    /* Pinmux */
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);

    /* UART start */
    uart_init(uart1, 115200);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8 ,1, 0);
	
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_init(LED1_PIN);
	gpio_set_dir(LED1_PIN, GPIO_OUT);

	multicore_launch_core1(core1_entry);

    sensors_init();

    char buf[200];
    
    sensor_data sd;

    while (1) {
        // update sensors
        if (sensors_update()) {
            sensors_read(&sd);
            sprintf(buf, "TIME: %ld R: %.2f P: %.2f Y: %.2f V: %.2f A: %.2f W: %.2f C°: %.1f lc: %ld %ld %ld %ld %ld %ld\n",
                get_time_ms() / 1000,
                sd.roll, sd.pitch, sd.yaw,
                sd.battery_voltage, sd.battery_current, sd.battery_power, sd.rp2040_temperature, 
                sd.load_cells_raw[0], sd.load_cells_raw[1], sd.load_cells_raw[2], sd.load_cells_raw[3], sd.load_cells_raw[4], sd.load_cells_raw[5]);
            tud_cdc_n_write_str(0, buf);
        }

        uint32_t len = tud_cdc_n_available(0);

        if (len) {
            uint8_t val;
            while (len--) {
                uart_putc(uart1, tud_cdc_n_read_char(0));
            }
        }
    }
}