const int PIEZO_PIN = 34;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);        // 0 ~ 4095
  analogSetAttenuation(ADC_11db);  // 0 ~ 약 3.3V 범위로 측정
}

void loop() {
  Serial.println(analogRead(PIEZO_PIN));
  delay(5);
}
