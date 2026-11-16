// SparkFun Weather Meter - 풍속계 단독 테스트 (ESP32)
const int ANEMO_PIN = 27;

volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseUs = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onAnemo() {
  unsigned long now = micros();
  if (now - lastPulseUs > 10000) {   // 10ms 디바운스
    portENTER_CRITICAL_ISR(&mux);
    pulseCount++;
    portEXIT_CRITICAL_ISR(&mux);
    lastPulseUs = now;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ANEMO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMO_PIN), onAnemo, FALLING);
  Serial.println("풍속계 테스트 시작");
}

void loop() {
  static unsigned long lastMs = 0;
  if (millis() - lastMs < 1000) return;   // 1초마다 출력
  float sec = (millis() - lastMs) / 1000.0;
  lastMs = millis();

  portENTER_CRITICAL(&mux);
  unsigned long count = pulseCount;
  pulseCount = 0;
  portEXIT_CRITICAL(&mux);

  float kmh = (count / sec) * 2.4;        // 1Hz = 2.4 km/h
  Serial.printf("펄스:%lu  풍속:%.2f m/s (%.1f km/h)\n", count, kmh / 3.6, kmh);
}
