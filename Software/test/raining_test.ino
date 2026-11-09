#define PLOT_MODE 0   // 0: 시리얼 모니터(문장),  1: 시리얼 플로터(그래프)

const int PIEZO_PIN = 34;   // 압전센서
const int RAIN_PIN  = 35;   // 빗물 감지 모듈 AO

// 압전센서 설정
const int PIEZO_THRESHOLD   = 300;  // 이 값을 넘으면 충격으로 판단
const int PIEZO_WINDOW_MS   = 20;   // 최고값을 모으는 시간
const int PIEZO_COOLDOWN_MS = 150;  // 감지 후 무시하는 시간

// 빗물센서 설정 (젖을수록 값이 작아짐)
const int RAIN_THRESHOLD   = 2500;  // 이 값보다 작으면 비가 온다고 판단
const int RAIN_INTERVAL_MS = 500;   // 0.5초마다 읽기

unsigned long lastHit = 0;
unsigned long lastRain = 0;
int rainValue = 4095;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

void loop() {
  unsigned long now = millis();
  int p = analogRead(PIEZO_PIN);

  // ── 빗물센서: 천천히 변하므로 0.5초마다 읽기 ──
  if (now - lastRain >= RAIN_INTERVAL_MS) {
    lastRain = now;
    rainValue = analogRead(RAIN_PIN);
#if !PLOT_MODE
    int wet = constrain(map(rainValue, 4095, 0, 0, 100), 0, 100);
    Serial.printf("[빗물] 값=%4d  젖음=%3d%%  → %s\n",
                  rainValue, wet, rainValue < RAIN_THRESHOLD ? "비 옴" : "건조");
#endif
  }

#if PLOT_MODE
  // ── 그래프 모드: 두 값을 계속 출력 ──
  Serial.printf("piezo:%d,rain:%d\n", p, rainValue);
  delay(5);
#else
  // ── 문장 모드: 압전 충격이 있을 때만 출력 ──
  if (p > PIEZO_THRESHOLD && now - lastHit > PIEZO_COOLDOWN_MS) {
    int peak = p;
    unsigned long t0 = millis();
    while (millis() - t0 < PIEZO_WINDOW_MS) {
      int s = analogRead(PIEZO_PIN);
      if (s > peak) peak = s;
    }
    lastHit = millis();
    Serial.printf("[압전] 충격 감지! 세기=%d\n", peak);
  }
#endif
}
