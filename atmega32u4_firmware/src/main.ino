const int FT4232_RESET_PIN = 12;

void setup(){
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);
    delay(5000);
    digitalWrite(12, HIGH);
}

void loop(){

}