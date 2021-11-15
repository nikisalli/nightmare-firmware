#include <sensors.h>

const int LOAD_CELL_TIMEOUT_MS = 10;
const uint8_t VOLTAGE_PIN = 26;
const uint8_t CURRENT_PIN = 27;
const uint8_t LOAD_CELL_GPIO[6] = {6, 7, 8, 9, 10, 11};
const uint8_t LOAD_CELL_CLK = 12;

gy953 mag = {
    .cs_pin = 17,
    .int_pin = 15,
    .sck_pin = 18,
    .mosi_pin = 19,
    .miso_pin = 16,
};

void sensors_init(){
    gy953_init(&mag);
    adc_init();
    adc_gpio_init(VOLTAGE_PIN);
    adc_gpio_init(CURRENT_PIN);
    adc_set_temp_sensor_enabled(true);

    // intialize load cells
    for (int i = 0; i < 6; i++){
        gpio_init(LOAD_CELL_GPIO[i]);
        gpio_set_dir(LOAD_CELL_GPIO[i], GPIO_IN);
    }

    gpio_init(LOAD_CELL_CLK);
    gpio_set_dir(LOAD_CELL_CLK, GPIO_OUT);
    gpio_set_drive_strength(LOAD_CELL_CLK, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(LOAD_CELL_CLK, GPIO_SLEW_RATE_FAST);
    gpio_put(LOAD_CELL_CLK, 0);
}

bool data_ready(){
    for (int i = 0; i < 6; i++){
        if (gpio_get(LOAD_CELL_GPIO[i]) == 1){
            return false;
        }
    }
    return true;
}

uint32_t get_time_ms(){
    return to_ms_since_boot(get_absolute_time());
}

bool sensors_update(){
    return gy953_update(&mag, 0);
}



// store function in ram to avoid slow loading from flash
void __attribute__((noinline, long_call, section(".time_critical"))) sensors_read(sensor_data* data){
    // read voltage and current
    adc_select_input(1);
    // Vin = (raw / (2^16 / 3.3)) / (R2 / (R1 + R2)) -> R1 = 10k, R2 = 1k
    data->battery_voltage_raw = adc_read();
    data->battery_voltage = (data->battery_voltage_raw / 1241.21) / (1000.0 / (10000.0 + 1000.0));
    adc_select_input(0);
    // Iin = (((raw / (2^16 / 3.3)) / (R2 / (R1 + R2))) - 2.5) * 10 -> R1 = 10k, R2 = 20k
    data->battery_current_raw = adc_read();
    data->battery_current = (((data->battery_current_raw / 1241.21) / (20000.0 / (10000.0 + 20000.0))) - 2.5) * 10;
    adc_select_input(4);
    // temp = (27 - (raw / (2^16 / 3.3) - 0.706) / 0.001721)
    data->rp2040_temperature_raw = adc_read();
    data->rp2040_temperature = 27.0 - ((data->rp2040_temperature_raw / 1241.21) - 0.706) / 0.001721;
    // calculate power
    data->battery_power = data->battery_voltage * data->battery_current;

    // read IMU
    int imu_data[3];
    gy953_getRPY(&mag, imu_data);
    data->roll_raw = imu_data[0];
    data->pitch_raw = imu_data[1];
    data->yaw_raw = imu_data[2];

    // compact inverse 2's complement
    data->roll = (data->roll_raw & (1 << 15)) != 0 ? data->roll_raw | ~((1 << 16) - 1) : data->roll_raw;
    data->pitch = (data->pitch_raw & (1 << 15)) != 0 ? data->pitch_raw | ~((1 << 16) - 1) : data->pitch_raw;
    data->yaw = (data->yaw_raw & (1 << 15)) != 0 ? data->yaw_raw | ~((1 << 16) - 1) : data->yaw_raw;

    // data is multiplied by 100 when transferred from the sensor
    data->roll /= 100.0;
    data->pitch /= 100.0;
    data->yaw /= 100.0;

    // wait for load cells to be ready
    uint32_t start_time = get_time_ms();
    while (!data_ready()){
        if (get_time_ms() - start_time > LOAD_CELL_TIMEOUT_MS){
            // skip reading load_cells if we timeout
            return;
        }
    }
    // read load cells
    // disable interrupts for time critical bit banging operations
    uint32_t interrupt_status = save_and_disable_interrupts();
    uint32_t buff[6] = {0};
    for (int i = 0; i < 24; i++){
        gpio_put(LOAD_CELL_CLK, 1);
        sleep_us(1);
        for (int j = 0; j < 6; j++){
            buff[j] |= (uint32_t)gpio_get(LOAD_CELL_GPIO[j]) << (23 - i);
        }
        gpio_put(LOAD_CELL_CLK, 0);
        sleep_us(1);
    }
    // another clock pulse to set gain to 128 on chA
    gpio_put(LOAD_CELL_CLK, 1);
    sleep_us(1);
    gpio_put(LOAD_CELL_CLK, 0);

    // restore interrupts
    restore_interrupts(interrupt_status);

    for (int i = 0; i < 6; i++){
        if ((buff[i] & (1 << 23)) != 0)
            data->load_cells_raw[i] = buff[i] | ~((1 << 24) - 1);
        else
            data->load_cells_raw[i] = buff[i];
    }
}