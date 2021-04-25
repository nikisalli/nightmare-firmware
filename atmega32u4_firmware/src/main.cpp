#include <Arduino.h>
#include <ArduinoJson.h>
#include <GY953.h>
#include <SPI.h>

const uint8_t FT4232_RESET_PIN = 12;
const uint8_t SERVO_CURRENT_PIN = A0;
const uint8_t COMPUTER_CURRENT_PIN = A2;
const uint8_t VOLTAGE_PIN = A1;
const uint8_t HEADER[5] = {0x55, 0x55, 0x55, 0x55, 0x55};
const uint8_t STAT_PACKET_ID = 0x00;

GY953 imu = GY953(3, 2);

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

class filter{
    public:
        float val;
        float raw_val;
        float filter_val;
        float min_input;
        float max_input;
        float min_output;
        float max_output;
        uint32_t gpio;
        
        filter(uint32_t gpio_, float filter_val_=0.99, float min_input_=0, float max_input_=1023, float min_output_=0, float max_output_=1023){
            gpio = gpio_;
            filter_val = filter_val_;
            min_input = min_input_;
            max_input = max_input_;
            min_output = min_output_;
            max_output = max_output_;
        }

        float update(){
            raw_val = fmap(analogRead(gpio), min_input, max_input, min_output, max_output);
            val += (raw_val - val) * (1.0 - filter_val);
            return val;
        }
};

filter servo_current(SERVO_CURRENT_PIN, 0.6, 512, 613.8, 0, 5);
filter computer_current(COMPUTER_CURRENT_PIN, 0.6, 512, 613.8, 0, 5);
filter voltage(VOLTAGE_PIN, 0.6, 0, 1023, 0, 55.55);
uint32_t prev_time = 0;

void setup(){
    Serial.begin(921600);
    imu.begin();
    pinMode(FT4232_RESET_PIN, OUTPUT);      //bootstrap usb chip to make sure it is recognized by the kernel
    digitalWrite(FT4232_RESET_PIN, LOW);
    delay(5000);
    digitalWrite(FT4232_RESET_PIN, HIGH);
}

void loop(){
    if (imu.update()) {
        // get data from IMU via spi
        int data[3];
        imu.getRPY(data);
    
        StaticJsonDocument<200> doc;
        doc["c"] = servo_current.update();      // servo current
        doc["rc"] = servo_current.raw_val;      // raw servo current
        doc["c1"] = computer_current.update();  // computer current
        doc["rc1"] = computer_current.raw_val;  // raw computer current
        doc["v"] = voltage.update();            // robot voltage
        doc["rv"] = voltage.raw_val;            // raw robot voltage
        doc["R"] = data[0];                     // imu roll
        doc["P"] = data[1];                     // imu pitch
        doc["Y"] = data[2];                     // imu yaw

        Serial.write(HEADER, sizeof(HEADER)); // HEADER LENGTH ID DATA
        uint16_t l = measureJson(doc) + 1; // json length + id length
        Serial.write(l >> 8); // upper
        Serial.write(l & 0xFF); // lower
        Serial.write(STAT_PACKET_ID);
        serializeJson(doc, Serial);
        prev_time = millis();
    }
}