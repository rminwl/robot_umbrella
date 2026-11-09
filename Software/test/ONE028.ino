const int rainPin = 35;  // ONE028 AO → GPIO35

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop() {
  int value = analogRead(rainPin);

  Serial.print("ONE028: ");
  Serial.println(value);

  delay(200);
}
