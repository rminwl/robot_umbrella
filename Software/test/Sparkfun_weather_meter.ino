// SparkFun Weather Meter - 풍속/풍향 테스트 (ESP32)
const int ANEMO_PIN = 27;     // 풍속계
const int VANE_PIN  = 34;     // 풍향계 (ADC1)
const float R_FIXED = 10000.0; // 분배 저항 10k
const float VCC_MV  = 3300.0;

volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseUs = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onAnemo() {
  unsigned long now = micros();
  if (now - lastPulseUs > 10000) {   // 10ms 디바운스 (채터링 제거)
    portENTER_CRITICAL_ISR(&mux);
    pulseCount++;
    portEXIT_CRITICAL_ISR(&mux);
    lastPulseUs = now;
  }
}

// 데이터시트의 16방향 저항값
const float vaneR[16]   = {33000, 6570, 8200, 891, 1000, 688, 2200, 1410,
                           3900, 3140, 16000, 14120, 120000, 42120, 64900, 21880};
const float vaneDeg[16] = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5,
                           180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5};
const char* vaneName[16] = {"N","NNE","NE","ENE","E","ESE","SE","SSE",
                            "S","SSW","SW","WSW","W","WNW","NW","NNW"};

void setup() {
  Serial.begin(115200);
  pinMode(ANEMO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMO_PIN), onAnemo, FALLING);
  analogReadResolution(12);
  Serial.println("Weather Meter 테스트 시작");
}

void loop() {
  static unsigned long lastMs = 0;
  if (millis() - lastMs < 1000) return;   // 1초마다 출력
  float sec = (millis() - lastMs) / 1000.0;
  lastMs = millis();

  // ---- 풍속 ----
  portENTER_CRITICAL(&mux);
  unsigned long count = pulseCount;
  pulseCount = 0;
  portEXIT_CRITICAL(&mux);
  float kmh = (count / sec) * 2.4;        // 1Hz = 2.4 km/h
  float ms  = kmh / 3.6;

  // ---- 풍향 ----
  int mv = analogReadMilliVolts(VANE_PIN);
  Serial.printf("펄스:%lu  풍속:%.2f m/s (%.1f km/h)  |  ADC:%d mV  ",
                count, ms, kmh, mv);

  if (mv > 3150) { Serial.println("→ 풍향계 미연결(개방)?"); return; }
  if (mv < 30)   { Serial.println("→ 풍향계 단락?");        return; }

  float r = R_FIXED * mv / (VCC_MV - mv);  // 측정 저항 역산
  int best = 0; float bestErr = 1e9;
  for (int i = 0; i < 16; i++) {           // 가장 가까운 방향 찾기
    float err = fabs(log(r / vaneR[i]));
    if (err < bestErr) { bestErr = err; best = i; }
  }
  Serial.printf("R:%.0fΩ  풍향:%s (%.1f°)\n", r, vaneName[best], vaneDeg[best]);
}