// SparkFun Weather Meter - 풍향계 단독 테스트 (ESP32)
const int VANE_PIN  = 34;      // ADC1 핀
const float R_FIXED = 10000.0; // 분배 저항 10k
const float VCC_MV  = 3300.0;

// 데이터시트의 16방향 저항값
const float vaneR[16]   = {33000, 6570, 8200, 891, 1000, 688, 2200, 1410,
                           3900, 3140, 16000, 14120, 120000, 42120, 64900, 21880};
const float vaneDeg[16] = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5,
                           180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5};
const char* vaneName[16] = {"N","NNE","NE","ENE","E","ESE","SE","SSE",
                            "S","SSW","SW","WSW","W","WNW","NW","NNW"};

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("풍향계 테스트 시작");
}

void loop() {
  int mv = analogReadMilliVolts(VANE_PIN);
  Serial.printf("ADC:%d mV  ", mv);

  if (mv > 3150) { Serial.println("→ 풍향계 미연결(개방)?"); delay(500); return; }
  if (mv < 30)   { Serial.println("→ 단락 또는 10kΩ 미연결?"); delay(500); return; }

  float r = R_FIXED * mv / (VCC_MV - mv);  // 풍향계 저항 역산
  int best = 0; float bestErr = 1e9;
  for (int i = 0; i < 16; i++) {           // 가장 가까운 방향 찾기
    float err = fabs(log(r / vaneR[i]));
    if (err < bestErr) { bestErr = err; best = i; }
  }
  Serial.printf("R:%.0fΩ  풍향:%s (%.1f°)\n", r, vaneName[best], vaneDeg[best]);
  delay(500);
}
