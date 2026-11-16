/*
 * wind_preprocess.ino
 * ---------------------------------------------------------------------------
 * 캡스톤 "환경적응형 웨어러블 로봇 우산" — 풍향·풍속 전처리 펌웨어
 *
 *   센서 원시값  →  정제된 바람 특징값(WindFeatures)  →  리브 8관절 목표각(RibTargets)
 *
 * - 대상: ESP32-DevKitC, Arduino IDE, arduino-esp32 core 3.x
 * - 센서: SparkFun Weather Meter (SEN-15901) 풍속계 + 풍향계
 * - 모터 전송 코드는 없다. RibTargets / jointDegToDxlTick() 결과를 나중에
 *   다이나믹셀 라이브러리에 넘기면 된다.
 * - [수정 가능성 있음] 모터 기종(XL330/XL430)과 개수(6~8개)는 미확정 → 파라미터 "리브 / 관절" 절 참고.
 *
 * [각도 규약]  ※ 이 파일 전체에서 동일
 *   - "풍향" = 바람이 "불어오는(FROM)" 방향. 변수명에 from 을 붙여 구분한다.
 *   - 상대 각도(rel) = 사용자 정면 0°, 위에서 내려다봤을 때 시계방향 증가.
 *       0° = 정면에서 불어옴, 90° = 오른쪽에서, 180° = 등 뒤에서, 270° = 왼쪽에서
 *   - 벡터 성분 (불어오는 방향 기준, 사용자 좌표계)
 *       u = v·sin θ  (+ 이면 오른쪽에서 불어오는 성분)
 *       w = v·cos θ  (+ 이면 정면에서 불어오는 성분)
 *   - 우산은 바람이 불어오는 쪽으로 기울인다 → tilt_dir_deg = 평활된 풍향(FROM)
 *
 * [참고] 센서가 사용자 몸에 붙어 있으므로 측정값은 "겉보기 바람"(실제 바람 +
 *        걷는 속도에 의한 맞바람)이다. 빗방울이 사용자에게 오는 방향도 겉보기
 *        바람 기준이므로 우산 제어에는 이 값이 오히려 맞다.
 *
 * 시리얼 명령(115200): 'h' = 헤더 재출력, 'w' = 넓게 모드, 'n' = 좁게 모드
 */

#include <Arduino.h>
#include <math.h>

// ============================================================================
// 1. 파라미터 (모든 조정값은 여기서만 바꾼다)
// ============================================================================

// ---- 핀 ----
constexpr int PIN_ANEMO        = 27;   // 풍속계 리드 스위치 (GPIO27 ↔ GND, 내부 풀업)
constexpr int PIN_VANE         = 34;   // 풍향계 전압 분배 출력 (ADC1, 입력 전용 핀)
constexpr int PIN_MODE_BUTTON  = 0;    // 캐노피 면적 모드 버튼 (기본: DevKitC BOOT 버튼, 누르면 LOW)
constexpr bool USE_MODE_BUTTON = true; // 버튼 사용 여부

// ---- 시리얼 / 주기 ----
constexpr uint32_t SERIAL_BAUD      = 115200; // 시리얼 속도 [bps]
constexpr uint32_t SAMPLE_PERIOD_MS = 100;    // 풍향 샘플링·전처리 주기 [ms] (10 Hz)
constexpr uint32_t OUTPUT_PERIOD_MS = 200;    // CSV 출력 주기 [ms] (5 Hz)
constexpr float    SAMPLE_HZ        = 1000.0f / SAMPLE_PERIOD_MS; // [Hz]

// ---- 풍속계 ----
constexpr float    ANEMO_MS_PER_HZ   = 2.4f / 3.6f; // 1초에 1회 닫힘 = 2.4 km/h = 0.667 [m/s per Hz]
constexpr uint32_t ANEMO_DEBOUNCE_US = 10000;       // 디바운스 [us] (10 ms)
constexpr uint32_t ANEMO_WINDOW_MS   = 1000;        // 펄스 수 집계 윈도우 [ms]
constexpr uint32_t ANEMO_TIMEOUT_MS  = 3000;        // 이 시간 동안 펄스가 없으면 순간 풍속 0 [ms]

// ---- 풍향계 ----
constexpr float VANE_R_PULLUP_OHM = 10000.0f; // 3.3V 쪽 고정 저항 [Ω]
constexpr float VANE_VCC_MV       = 3300.0f;  // 분배 전원 전압 [mV] (멀티미터로 실측해 넣으면 더 정확)
// [실험 후 보정] 단락 기준은 "단락 실측값"과 "가장 낮은 정상값(688Ω, 112.5° 위치, 이론 212mV)"의
//   가운데로 둔다. 단락 실측 142mV 기준이면 (142+212)/2 ≈ 177 → 180 유지.
//   112.5° 위치 실측값이 200mV 이하로 나오면 여유가 줄어드니 그 값으로 다시 계산할 것.
constexpr int   VANE_SHORT_MV     = 180;      // 이 값 미만 = 단락 [mV] (ESP32 ADC는 0V를 약 140mV로 읽음)
constexpr int   VANE_OPEN_MV      = 3100;     // ADC 측정 상한 이상은 미연결/포화 (실측 보정 필요)
constexpr float VANE_MAX_LOG_ERR  = 0.20f;    // |ln(R측정/R테이블)| 허용치 (0.20 ≈ ±20%), 초과 시 무효
constexpr int   VANE_OVERSAMPLE   = 4;        // 1회 샘플당 ADC 평균 횟수 (전압끼리 평균, 각도 평균 아님)
// 풍향 오류 처리 단계
//   0 ~ VANE_FAULT_TIMEOUT_MS : 직전 유효 방향을 유지하며 추적 계속 (valid=0, sensor_fault=0)
//   그 이후                   : sensor_fault=1, direction_valid=0 → 추적 중단, 관절은 FAULT_ACTION 자세
//   정상 판독이 다시 들어오면 sensor_fault=0 으로 즉시 복귀 (방향은 평활창이 채워지며 회복)
constexpr uint32_t VANE_FAULT_TIMEOUT_MS = 2000; // 풍향 무효가 이 시간 넘게 지속되면 센서 고장 [ms]
enum FaultAction : uint8_t { FAULT_TO_NEUTRAL = 0, FAULT_TO_SAFE = 1 };
// [실험 후 조정] 센서 고장 시 관절 자세: FAULT_TO_NEUTRAL = 현재 캐노피 모드 중립, FAULT_TO_SAFE = 강풍 안전 자세
//   바람 정보가 없을 때 강풍일 수도 있다고 보면 SAFE, 사용자 시야·편의를 우선하면 NEUTRAL.
//   ※ 풍속계(리드 스위치) 고장은 "펄스 없음 = 무풍"과 구분되지 않아 감지하지 못한다.
constexpr FaultAction FAULT_ACTION = FAULT_TO_NEUTRAL;

// 16방향 저항표 [Ω] — 0°부터 22.5° 간격 (풍향계 자체의 N 표시 기준, 시계방향)
constexpr int   VANE_NUM_DIRS = 16;
const float VANE_TABLE_OHM[VANE_NUM_DIRS] = {
  33000, 6570, 8200, 891, 1000, 688, 2200, 1410,
  3900, 3140, 16000, 14120, 120000, 42120, 64900, 21880
};

// ---- 장착 ----
// 사용자 정면(0°)을 기준으로, 풍향계 N 표시가 가리키는 방향 [deg, 시계방향 +]
//   N 표시가 사용자 정면을 향하면 0, 오른쪽이면 90, 등 뒤면 180, 왼쪽이면 270
constexpr float MOUNT_OFFSET_DEG = 0.0f;
constexpr float VANE_DIR_SIGN    = +1.0f;  // 풍향계를 뒤집어 달아 각도가 반시계로 증가하면 -1

// ---- 평활화 ----
constexpr bool  SMOOTH_USE_EMA = false; // false = 이동창 평균, true = 지수이동평균(EMA)
// [실험 후 조정] 이동창은 평균 지연이 약 W/2 (3초 → 약 1.5초 늦음). speed_avg 는 기울기 크기·무풍 판정에 쓰인다.
//   강풍 모드는 아래 돌풍값으로도 판정하므로 이 창을 늘려도 강풍 반응은 늦어지지 않는다.
//   더 빠른 반응이 필요하면 2초 또는 EMA(시정수 1초) 로 바꿔 선풍기 시험에서 떨림과 비교할 것.
constexpr float AVG_WINDOW_S   = 3.0f;  // 이동창 길이 [s]
constexpr float EMA_TAU_S      = 1.5f;  // EMA 시정수 [s] (이동창 W 와 비슷하게 하려면 약 W/2)
constexpr float GUST_AVG_S     = 1.0f;  // 돌풍용 단기 평균 길이 [s] — 펄스 1개짜리 튐(채터링)이 돌풍으로 잡히지 않게 함
constexpr float GUST_WINDOW_S  = 10.0f; // 돌풍 = 최근 이 시간 동안 "GUST_AVG_S 평균 풍속"의 최대값 [s]

// ---- 무풍 / 방향 안정도 ----
constexpr float CALM_THRESHOLD_MS = 1.0f;  // 평균 풍속이 이 값 미만이면 무풍 [m/s]
constexpr float CALM_HYST_MS      = 0.3f;  // 무풍 해제는 CALM_THRESHOLD + 이 값 초과일 때 [m/s]
constexpr float STABILITY_MIN_R   = 0.30f; // 안정도 R이 이 값 미만이면 방향 무효 (0~1)
constexpr float STABILITY_HYST_R  = 0.10f; // 방향 유효 복귀는 STABILITY_MIN_R + 이 값 초과일 때

// ---- 보호 섹터 ----
constexpr float SECTOR_HYST_DEG = 5.0f; // 섹터 경계 히스테리시스 [deg] (±22.5° + 이 값을 넘어야 섹터 변경)

// ---- 규칙 기반 기울기 ----
constexpr float V_RAIN_MS    = 6.0f;  // 빗방울 낙하 속도 [m/s]
constexpr float TILT_MAX_DEG = 30.0f; // 최대 기울기 [deg]

// ---- 강풍(안전) 모드 ----
// [실험 후 조정] 10 m/s 는 일반 우산이 뒤집히기 시작하는 수준(약 10~14 m/s)을 참고한 초기값.
//   리브·관절 강도 시험(선풍기/송풍기)에서 변형이 시작되는 풍속의 70~80% 로 정할 것.
constexpr float    WIND_SAFE_LIMIT_MS = 10.0f; // 평균 풍속 또는 돌풍이 이 값 초과 → 강풍 모드 [m/s]
constexpr float    WIND_SAFE_HYST_MS  = 1.5f;  // 해제는 (한계 − 이 값) 미만일 때 [m/s]
constexpr uint32_t SAFE_RELEASE_MS    = 5000;  // 해제 조건이 이 시간 동안 유지돼야 해제 [ms]

// ---- 리브 / 관절 ----
// 리브 구성 (Design/motor_rib_1111_Modularization 기준, 등에 메는 하우징)
//   r1 = Top_L  : 하우징 위쪽 → 머리 위 앞왼쪽
//   r2 = Top_R  : 하우징 위쪽 → 머리 위 앞오른쪽
//   r3 = Side_L : 하우징 아래 왼쪽
//   r4 = Side_R : 하우징 아래 오른쪽
//   ※ 등 뒤(180°)를 직접 막는 리브는 없다. 뒤에서 오는 바람은 c_i ≤ 0 이라
//     바람 쪽으로 내리는 리브가 없고, 윗리브만 살짝 올라간다.
//
// 관절 각도 규약: 0° = 기준 자세(리브 펼침), + = 위로, − = 아래로 [deg]
//   J1: 리브 뿌리, 리브 전체 올림(+)/내림(−)
//   J2: 리브 중간, 끝부분 펴기(+)/아래로 굽힘(−)
// [수정 가능성 있음] 관절 구조 미확정.
//   - 옆리브(Side)는 세로축 핀 + 평행사변형 링크라 J1이 위아래가 아니라 수평 회전일 수 있다.
//     확정되면 옆리브 J1 의 부호/게인을 이 규약에 맞게 다시 정할 것.
//   - 윗리브 끝(Top Rib3)은 미리 굽힌 고정 형상일 수 있다 → 그 경우 J2 를 JOINT_PRESENT 에서 끈다.
constexpr int NUM_RIBS = 4;
const char* const RIB_NAME[NUM_RIBS] = {"Top_L", "Top_R", "Side_L", "Side_R"};
const float RIB_ANGLE_DEG[NUM_RIBS] = {315, 45, 270, 90}; // 리브가 덮는 방향 [deg] (정면 0°, 시계방향) — 설계 이미지 기준 추정값

// [수정 가능성 있음] 모터 수 미확정 (예상 6~8개).
//   실제로 모터가 달린 관절만 true. 없는 관절은 계산은 하지만 지령/출력하지 않는다(CSV 빈 칸).
//   예) 8개: 모두 true
//       6개: 윗리브 J1·J2 + 옆리브 J1만 → {{1,1},{1,1},{1,0},{1,0}}
const bool JOINT_PRESENT[NUM_RIBS][2] = {
  {true, true},   // Top_L  {J1, J2}
  {true, true},   // Top_R
  {true, true},   // Side_L
  {true, true},   // Side_R
};

constexpr float J1_GAIN_DOWN = 25.0f; // 바람 쪽 리브 J1 최대 내림량 [deg] (c=1, s=1 일 때)
constexpr float J2_GAIN_DOWN = 30.0f; // 바람 쪽 리브 J2 최대 굽힘량 [deg]
constexpr float J1_GAIN_UP   = 10.0f; // 반대쪽 리브 J1 최대 올림량 [deg]

constexpr float J1_MIN = -45.0f, J1_MAX = 30.0f; // J1 허용 범위 [deg]
constexpr float J2_MIN = -60.0f, J2_MAX = 10.0f; // J2 허용 범위 [deg]

constexpr float J1_SAFE = -35.0f; // 강풍 모드 안전 자세 J1 [deg] (리브를 접어 면적 최소화)
constexpr float J2_SAFE = -45.0f; // 강풍 모드 안전 자세 J2 [deg]

const float JOINT_MAX_SPEED_DPS[2] = {60.0f, 90.0f}; // 관절별 최대 회전 속도 {J1, J2} [deg/s]
constexpr float DEADBAND_DEG = 2.0f; // 목표각 변화가 이 값 미만이면 목표 갱신 안 함 [deg]

// 캐노피 면적 모드별 중립 자세 {J1, J2} [deg]
enum CanopyMode : uint8_t { MODE_WIDE = 0, MODE_NARROW = 1, NUM_MODES };
const char* const MODE_NAME[NUM_MODES] = {"WIDE", "NARROW"};
const float NEUTRAL_POSE[NUM_MODES][2] = {
  {  0.0f,   0.0f},  // WIDE  : 넓게 펼침
  {-15.0f, -20.0f},  // NARROW: 좁게 (리브를 내리고 끝을 굽힘)
};

// ---- 다이나믹셀 위치 변환 ----
// [수정 가능성 있음] 모터 기종 미확정: XL330-M288-T(5V, 약 18g) 또는 XL430-W250-T(11.1V, 약 57g, 현재 설계 모델).
//   두 기종 모두 Goal Position 0~4095, 1 단위 = 0.088° 라 아래 변환은 그대로 쓸 수 있다.
//   전원 전압·전류, 무게, 토크는 기종에 따라 크게 달라지므로 기종을 확정하면 하드웨어 쪽을 다시 확인할 것.
//   다른 계열(예: 해상도가 다른 모델)로 바꾸면 DXL_DEG_PER_TICK, DXL_TICK_MAX 를 수정.
constexpr float DXL_DEG_PER_TICK = 0.088f; // Goal Position 1 단위 [deg]
constexpr int   DXL_TICK_MIN = 0, DXL_TICK_MAX = 4095;
// 관절별 영점 오프셋: 관절각 0° 일 때의 Goal Position 값 [tick] — {J1, J2} × 리브 4개
int16_t JOINT_ZERO_TICK[NUM_RIBS][2] = {
  {2048, 2048}, {2048, 2048}, {2048, 2048}, {2048, 2048}
};
// 관절별 회전 방향 부호: 관절각 +(위로)가 모터 tick 증가면 +1, 감소면 -1
int8_t JOINT_DIR_SIGN[NUM_RIBS][2] = {
  {+1, +1}, {+1, +1}, {+1, +1}, {+1, +1}
};

// ============================================================================
// 2. 내부 상수와 자료구조
// ============================================================================

constexpr float DEG2RAD = (float)M_PI / 180.0f;
constexpr float RAD2DEG = 180.0f / (float)M_PI;
constexpr int AVG_N  = (int)(AVG_WINDOW_S  * SAMPLE_HZ + 0.5f); // 이동창 샘플 수
constexpr int GUST_N = (int)(GUST_WINDOW_S * SAMPLE_HZ + 0.5f); // 돌풍창 샘플 수
constexpr int GUST_AVG_N = (int)(GUST_AVG_S * SAMPLE_HZ + 0.5f); // 돌풍용 단기 평균 샘플 수
static_assert(AVG_N >= 1 && GUST_N >= 1 && GUST_AVG_N >= 1, "window too short");

enum VaneStatus : uint8_t { VANE_OK = 0, VANE_SHORT, VANE_OPEN, VANE_OUT_OF_TABLE };
const char* const VANE_STATUS_NAME[] = {"OK", "SHORT", "OPEN", "RANGE"};

// 보호 섹터 (상대 풍향 FROM 기준 8방위)
const char* const SECTOR_NAME[8] = {
  "F",   // 0 앞
  "FR",  // 1 앞오른쪽
  "R",   // 2 오른쪽
  "BR",  // 3 뒤오른쪽
  "B",   // 4 뒤
  "BL",  // 5 뒤왼쪽
  "L",   // 6 왼쪽
  "FL",  // 7 앞왼쪽
};
constexpr int SECTOR_NONE = -1; // 막을 방향 없음

// 풍향계 1회 판독 결과
struct VaneReading {
  VaneStatus status;
  int   adc_mv;          // 측정 전압 [mV]
  float r_ohm;           // 역산 저항 [Ω] (단락 0, 미연결 -1)
  int   index;           // 가장 가까운 테이블 인덱스 (0~15), 무효면 -1
  float log_err;         // |ln(R측정/R테이블)|
  float from_raw_deg;    // 풍향계 기준 불어오는 방향 [deg] (무효면 의미 없음)
};

// 전처리 결과 — 모터 제어/학습 모두 이 struct 하나로 넘긴다
struct WindFeatures {
  uint32_t t_ms;
  bool  valid;              // 이번 풍향 샘플이 유효한가 (무효면 아래 dir_* 는 직전 유효값 유지)
  VaneStatus vane_status;
  int   adc_mv;
  float vane_r_ohm;
  float dir_from_raw_deg;   // 풍향계 기준 불어오는 방향 [deg]
  float dir_from_rel_deg;   // 사용자 기준 불어오는 방향 (순간) [deg]
  float speed_inst_ms;      // 펄스 간격 기반 순간 풍속 [m/s]
  float speed_win_ms;       // 1초 윈도우 펄스 수 기반 풍속 [m/s]
  float speed_avg_ms;       // 평활 풍속 [m/s]
  float gust_ms;            // 최근 GUST_WINDOW_S 동안 GUST_AVG_S 평균 풍속의 최대값 [m/s]
  float u_ms;               // 평활 벡터 u = v·sinθ [m/s]
  float w_ms;               // 평활 벡터 w = v·cosθ [m/s]
  float dir_from_avg_deg;   // 평활된 상대 풍향(FROM) [deg]
  float sin_dir;            // sin(dir_from_avg)
  float cos_dir;            // cos(dir_from_avg)
  float stability_R;        // 풍향 안정도 (0~1, 1 = 한 방향으로 일정)
  bool  direction_valid;    // false = 무풍/불안정/센서 고장 → 막을 방향 없음
  int   protect_sector;     // 0~7, SECTOR_NONE(-1)
  float tilt_dir_deg;       // 기울일 방향 = 평활 풍향(FROM) [deg]
  float tilt_deg_baseline;  // 규칙 기반 기울기 크기 [deg] (방향 무효면 0)
  bool  safe_mode;          // 강풍 모드
  // ---- 센서 오류 기록 (10 Hz 처리 기준 누적값이라 5 Hz 출력 사이의 짧은 오류도 빠지지 않음) ----
  bool     sensor_fault;      // 풍향 오류가 VANE_FAULT_TIMEOUT_MS 넘게 지속 → 추적 중단
  uint32_t err_events;        // 오류 발생 횟수 (정상 → 오류 전환 수)
  uint32_t err_samples;       // 오류 샘플 수 (10 Hz)
  uint32_t err_total_ms;      // 누적 오류 시간 [ms]
  uint32_t err_cur_ms;        // 현재 진행 중인 오류 지속 시간 [ms] (정상이면 0)
  uint32_t err_max_ms;        // 가장 길었던 오류 지속 시간 [ms]
};

// 리브 8관절 목표각 [deg] — j[리브][0]=J1, j[리브][1]=J2
struct RibTargets {
  float j[NUM_RIBS][2];
};

// ============================================================================
// 3. 풍속계 ISR
// ============================================================================

portMUX_TYPE g_anemoMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t g_pulseCount      = 0; // 누적 펄스 수
volatile uint32_t g_lastPulseUs     = 0; // 마지막 유효 펄스 시각 [us]
volatile uint32_t g_pulseIntervalUs = 0; // 직전 두 펄스 간격 [us] (0 = 아직 없음)
volatile bool g_havePulse = false;

void IRAM_ATTR onAnemoPulse() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&g_anemoMux);
  uint32_t dt = now - g_lastPulseUs;
  if (!g_havePulse || dt >= ANEMO_DEBOUNCE_US) {
    g_pulseIntervalUs = (g_havePulse && dt <= ANEMO_TIMEOUT_MS * 1000UL) ? dt : 0;
    g_havePulse = true;
    g_lastPulseUs = now;
    g_pulseCount = g_pulseCount + 1;
  }
  portEXIT_CRITICAL_ISR(&g_anemoMux);
}

// ============================================================================
// 4. 순수 함수들 (상태 없음 — 테스트와 Python 포팅이 쉬움)
// ============================================================================

float wrap360(float deg) {
  float r = fmodf(deg, 360.0f);
  if (r < 0) r += 360.0f;
  return (r >= 360.0f) ? 0.0f : r;   // -1e-7 + 360 이 360.0 으로 반올림되는 경우 방지
}

// a − b 를 −180~180 으로
float angDiff(float a, float b) {
  float d = wrap360(a - b);
  return (d > 180.0f) ? d - 360.0f : d;
}

// 전압 → 풍향 판독
VaneReading decodeVane(int mv) {
  VaneReading r = {VANE_OK, mv, 0.0f, -1, 0.0f, 0.0f};
  if (mv < VANE_SHORT_MV) { r.status = VANE_SHORT; r.r_ohm = 0.0f;  return r; }
  if (mv >= VANE_OPEN_MV || mv >= VANE_VCC_MV) { r.status = VANE_OPEN; r.r_ohm = -1.0f; return r; }

  // 분배식 V = Vcc · R / (R + Rpullup)  →  R = Rpullup · V / (Vcc − V)
  r.r_ohm = VANE_R_PULLUP_OHM * mv / (VANE_VCC_MV - mv);

  float bestErr = 1e9f;
  float lnR = logf(r.r_ohm);
  for (int i = 0; i < VANE_NUM_DIRS; i++) {
    float e = fabsf(lnR - logf(VANE_TABLE_OHM[i])); // log 비율 = 저항비 기준 거리
    if (e < bestErr) { bestErr = e; r.index = i; }
  }
  r.log_err = bestErr;
  r.from_raw_deg = r.index * 22.5f;
  if (bestErr > VANE_MAX_LOG_ERR) { r.status = VANE_OUT_OF_TABLE; r.index = -1; }
  return r;
}

// 풍향계 기준 각도 → 사용자 기준 상대 각도 (둘 다 불어오는 방향)
float vaneToUserFromDeg(float fromRawDeg) {
  return wrap360(VANE_DIR_SIGN * fromRawDeg + MOUNT_OFFSET_DEG);
}

// 가중 원형 평균. 각도를 산술 평균하지 않고 단위벡터 × 가중치의 합으로 구한다.
//   meanDeg: 평균 방향, R: 가중 결과벡터 길이 / 가중치 합 (0~1)
//   가중치 합이 0이면 false
bool circularMeanDeg(const float* deg, const float* weight, int n, float& meanDeg, float& R) {
  float su = 0, sw = 0, sv = 0;
  for (int i = 0; i < n; i++) {
    su += weight[i] * sinf(deg[i] * DEG2RAD);
    sw += weight[i] * cosf(deg[i] * DEG2RAD);
    sv += weight[i];
  }
  if (sv <= 1e-6f) { meanDeg = 0; R = 0; return false; }
  meanDeg = wrap360(atan2f(su, sw) * RAD2DEG);
  R = sqrtf(su * su + sw * sw) / sv;
  return true;
}

// 상대 풍향 → 8방위 섹터 (prevSector 기준 히스테리시스)
int sectorFromDir(float dirDeg, int prevSector) {
  if (prevSector >= 0 &&
      fabsf(angDiff(dirDeg, prevSector * 45.0f)) <= 22.5f + SECTOR_HYST_DEG) {
    return prevSector;
  }
  return ((int)floorf(wrap360(dirDeg + 22.5f) / 45.0f)) % 8;
}

// 규칙 기반 기울기: 빗줄기 기울기 = atan(풍속 / 빗방울 낙하 속도)
float baselineTiltDeg(float speedMs) {
  float t = atanf(speedMs / V_RAIN_MS) * RAD2DEG;
  return (t > TILT_MAX_DEG) ? TILT_MAX_DEG : t;
}

// 리브 관절 목표각 매핑 (순수 함수)
//   tiltDirDeg : 기울일 방향 = 바람이 불어오는 방향 [deg] (사용자 기준)
//   tiltDeg    : 기울기 크기 [deg] (0~TILT_MAX_DEG)
//   학습 모델 출력(target_tilt_dir_deg, target_tilt_deg)도 그대로 넣을 수 있다.
RibTargets mapRibTargets(float tiltDirDeg, float tiltDeg, bool directionValid,
                         CanopyMode mode, bool safeMode) {
  RibTargets t;
  const float n1 = NEUTRAL_POSE[mode][0];
  const float n2 = NEUTRAL_POSE[mode][1];
  float s = tiltDeg / TILT_MAX_DEG;
  s = constrain(s, 0.0f, 1.0f);

  for (int i = 0; i < NUM_RIBS; i++) {
    float j1, j2;
    if (safeMode) {                 // 강풍: 기울이지 않고 안전 자세
      j1 = J1_SAFE; j2 = J2_SAFE;
    } else if (!directionValid) {   // 무풍/불안정: 중립
      j1 = n1; j2 = n2;
    } else {
      float c = cosf((RIB_ANGLE_DEG[i] - tiltDirDeg) * DEG2RAD); // 정렬도 −1~1
      if (c > 0) {                  // 바람 쪽: 내려서 비스듬한 비를 막음
        j1 = n1 - J1_GAIN_DOWN * c * s;
        j2 = n2 - J2_GAIN_DOWN * c * s;
      } else {                      // 반대쪽: 살짝 올려 바람 받는 면적 축소
        j1 = n1 + J1_GAIN_UP * (-c) * s;
        j2 = n2;
      }
    }
    t.j[i][0] = constrain(j1, J1_MIN, J1_MAX);
    t.j[i][1] = constrain(j2, J2_MIN, J2_MAX);
  }
  return t;
}

// 관절각 [deg] → 다이나믹셀 Goal Position [tick] (XL330/XL430 공통)
uint16_t jointDegToDxlTick(int rib, int joint, float deg) {
  float tick = JOINT_ZERO_TICK[rib][joint] + JOINT_DIR_SIGN[rib][joint] * deg / DXL_DEG_PER_TICK;
  long v = lroundf(tick);
  return (uint16_t)constrain(v, (long)DXL_TICK_MIN, (long)DXL_TICK_MAX);
}

// ============================================================================
// 5. 상태가 있는 처리 (평활, 히스테리시스, 속도 제한)
// ============================================================================

// ---- 관절 목표 데드밴드 + 속도 제한 ----
struct JointLimiter {
  RibTargets held;  // 데드밴드를 통과한 목표
  RibTargets cmd;   // 속도 제한된 실제 지령 (모터에 보낼 값)
  bool init = false;

  const RibTargets& update(const RibTargets& target, float dtS) {
    for (int i = 0; i < NUM_RIBS; i++) {
      for (int k = 0; k < 2; k++) {
        if (!init) { held.j[i][k] = cmd.j[i][k] = target.j[i][k]; continue; }
        if (fabsf(target.j[i][k] - held.j[i][k]) >= DEADBAND_DEG) held.j[i][k] = target.j[i][k];
        float maxStep = JOINT_MAX_SPEED_DPS[k] * dtS;
        float d = constrain(held.j[i][k] - cmd.j[i][k], -maxStep, maxStep);
        cmd.j[i][k] += d;
      }
    }
    init = true;
    return cmd;
  }
};

// ---- 전처리 상태 ----
struct WindState {
  // 이동창 / 돌풍창 링버퍼
  float bufU[AVG_N], bufW[AVG_N], bufV[AVG_N];
  int   avgHead = 0, avgCount = 0;
  float bufGustAvg[GUST_AVG_N];
  int   gustAvgHead = 0, gustAvgCount = 0;
  float bufGust[GUST_N];
  int   gustHead = 0, gustCount = 0;
  // 오류 기록
  bool     prevValid = true;
  uint32_t errStartMs = 0;
  uint32_t errEvents = 0, errSamples = 0, errTotalMs = 0, errMaxMs = 0;
  // EMA
  float emaU = 0, emaW = 0, emaV = 0;
  bool  emaInit = false;
  // 풍향 유지값
  bool  everValid = false;
  float lastFromRawDeg = 0, lastFromRelDeg = 0;
  uint32_t lastValidMs = 0;
  float lastTiltDir = 0;
  // 히스테리시스 상태
  bool  calm = true, unstable = true, safe = false;
  uint32_t safeReleaseStartMs = 0; bool safeReleasing = false;
  int   sector = SECTOR_NONE;
  // 풍속 윈도우
  uint32_t winStartMs = 0, winStartCount = 0;
  float speedWin = 0;
};

WindState    g_ws;
WindFeatures g_feat;
RibTargets   g_ribCmd;
JointLimiter g_limiter;
CanopyMode   g_mode = MODE_WIDE;

int readVaneMv() {
  long sum = 0;
  for (int i = 0; i < VANE_OVERSAMPLE; i++) sum += analogReadMilliVolts(PIN_VANE);
  return (int)(sum / VANE_OVERSAMPLE);
}

void anemoSnapshot(uint32_t& count, uint32_t& lastUs, uint32_t& intervalUs) {
  portENTER_CRITICAL(&g_anemoMux);
  // 만료된 간격을 지워 micros() 순환(약 71분) 후 과거 풍속이 부활하지 않게 한다.
  if (g_havePulse && (uint32_t)(micros() - g_lastPulseUs) > ANEMO_TIMEOUT_MS * 1000UL) {
    g_havePulse = false;
    g_pulseIntervalUs = 0;
  }
  count = g_pulseCount; lastUs = g_lastPulseUs; intervalUs = g_pulseIntervalUs;
  portEXIT_CRITICAL(&g_anemoMux);
}

// 10 Hz 전처리 1스텝
void processSample(uint32_t nowMs) {
  WindState& s = g_ws;
  WindFeatures& f = g_feat;
  f.t_ms = nowMs;

  // ---- 풍속 ----
  uint32_t cnt, lastUs, intUs;
  anemoSnapshot(cnt, lastUs, intUs);
  uint32_t nowUs = micros();
  uint32_t sinceUs = nowUs - lastUs;
  if (intUs == 0 || sinceUs > ANEMO_TIMEOUT_MS * 1000UL) {
    f.speed_inst_ms = 0.0f;
  } else {
    // 마지막 펄스 이후 경과시간이 직전 간격보다 길면 그만큼 느려진 것으로 본다 (부드러운 감쇠)
    float effUs = (float)max(intUs, sinceUs);
    f.speed_inst_ms = ANEMO_MS_PER_HZ * 1e6f / effUs;
  }
  uint32_t winElapsed = nowMs - s.winStartMs;
  if (winElapsed >= ANEMO_WINDOW_MS) {
    s.speedWin = ANEMO_MS_PER_HZ * (cnt - s.winStartCount) * 1000.0f / winElapsed;
    s.winStartMs = nowMs; s.winStartCount = cnt;
  }
  f.speed_win_ms = s.speedWin;

  // ---- 풍향 ----
  VaneReading vr = decodeVane(readVaneMv());
  f.vane_status = vr.status;
  f.adc_mv = vr.adc_mv;
  f.vane_r_ohm = vr.r_ohm;
  f.valid = (vr.status == VANE_OK);
  if (f.valid) {
    s.lastFromRawDeg = vr.from_raw_deg;
    s.lastFromRelDeg = vaneToUserFromDeg(vr.from_raw_deg);
    s.lastValidMs = nowMs;
    s.everValid = true;
  }
  f.dir_from_raw_deg = s.lastFromRawDeg;  // 무효면 이전 유효값 유지
  f.dir_from_rel_deg = s.lastFromRelDeg;

  // ---- 오류 기록 / 센서 고장 판정 ----
  if (!f.valid) {
    if (s.prevValid) { s.errEvents++; s.errStartMs = nowMs; }
    s.errSamples++;
    s.errTotalMs += SAMPLE_PERIOD_MS;
    f.err_cur_ms = nowMs - s.errStartMs + SAMPLE_PERIOD_MS;
    if (f.err_cur_ms > s.errMaxMs) s.errMaxMs = f.err_cur_ms;
  } else {
    f.err_cur_ms = 0;
  }
  s.prevValid = f.valid;
  f.err_events = s.errEvents; f.err_samples = s.errSamples;
  f.err_total_ms = s.errTotalMs; f.err_max_ms = s.errMaxMs;
  f.sensor_fault = !f.valid && f.err_cur_ms > VANE_FAULT_TIMEOUT_MS;
  bool vaneFresh = s.everValid && !f.sensor_fault;

  // 고장 중 가상의 일정 방향을 누적하지 않고, 복구 시 새 측정으로 시작한다.
  if (!vaneFresh) {
    for (int i = 0; i < AVG_N; i++) { s.bufU[i] = 0; s.bufW[i] = 0; }
    s.emaU = s.emaW = 0;
  }

  // ---- 풍속 가중 벡터 ----
  float v  = f.speed_inst_ms;
  float th = f.dir_from_rel_deg * DEG2RAD;
  float u = vaneFresh ? v * sinf(th) : 0.0f;
  float w = vaneFresh ? v * cosf(th) : 0.0f;

  float uA, wA, vA;
  if (SMOOTH_USE_EMA) {
    float a = (1.0f / SAMPLE_HZ) / (EMA_TAU_S + 1.0f / SAMPLE_HZ);
    if (!s.emaInit) { s.emaU = u; s.emaW = w; s.emaV = v; s.emaInit = true; }
    s.emaU += a * (u - s.emaU); s.emaW += a * (w - s.emaW); s.emaV += a * (v - s.emaV);
    uA = s.emaU; wA = s.emaW; vA = s.emaV;
  } else {
    s.bufU[s.avgHead] = u; s.bufW[s.avgHead] = w; s.bufV[s.avgHead] = v;
    s.avgHead = (s.avgHead + 1) % AVG_N;
    if (s.avgCount < AVG_N) s.avgCount++;
    float su = 0, sw = 0, sv = 0;
    for (int i = 0; i < s.avgCount; i++) { su += s.bufU[i]; sw += s.bufW[i]; sv += s.bufV[i]; }
    uA = su / s.avgCount; wA = sw / s.avgCount; vA = sv / s.avgCount;
  }
  f.u_ms = uA; f.w_ms = wA; f.speed_avg_ms = vA;
  float mag = sqrtf(uA * uA + wA * wA);
  f.stability_R = (vA > 1e-3f) ? constrain(mag / vA, 0.0f, 1.0f) : 0.0f;
  if (mag > 1e-4f) f.dir_from_avg_deg = wrap360(atan2f(uA, wA) * RAD2DEG);
  // (벡터가 0이면 직전 평균 방향 유지)
  f.sin_dir = sinf(f.dir_from_avg_deg * DEG2RAD);
  f.cos_dir = cosf(f.dir_from_avg_deg * DEG2RAD);

  // ---- 돌풍: 순간 풍속의 GUST_AVG_S 평균 → 최근 GUST_WINDOW_S 최대 ----
  s.bufGustAvg[s.gustAvgHead] = v;
  s.gustAvgHead = (s.gustAvgHead + 1) % GUST_AVG_N;
  if (s.gustAvgCount < GUST_AVG_N) s.gustAvgCount++;
  float vShort = 0;
  for (int i = 0; i < s.gustAvgCount; i++) vShort += s.bufGustAvg[i];
  vShort /= s.gustAvgCount;
  s.bufGust[s.gustHead] = vShort;
  s.gustHead = (s.gustHead + 1) % GUST_N;
  if (s.gustCount < GUST_N) s.gustCount++;
  float g = 0;
  for (int i = 0; i < s.gustCount; i++) g = max(g, s.bufGust[i]);
  f.gust_ms = g;

  // ---- 무풍 / 안정도 히스테리시스 ----
  if (s.calm) { if (vA > CALM_THRESHOLD_MS + CALM_HYST_MS) s.calm = false; }
  else        { if (vA < CALM_THRESHOLD_MS) s.calm = true; }
  if (s.unstable) { if (f.stability_R > STABILITY_MIN_R + STABILITY_HYST_R) s.unstable = false; }
  else            { if (f.stability_R < STABILITY_MIN_R) s.unstable = true; }
  f.direction_valid = !s.calm && !s.unstable && vaneFresh;

  // ---- 보호 섹터 / 기준 기울기 ----
  if (f.direction_valid) {
    s.sector = sectorFromDir(f.dir_from_avg_deg, s.sector);
    s.lastTiltDir = f.dir_from_avg_deg;
    f.tilt_deg_baseline = baselineTiltDeg(vA);
  } else {
    s.sector = SECTOR_NONE;
    f.tilt_deg_baseline = 0.0f;     // 막을 방향 없음
  }
  f.protect_sector = s.sector;
  f.tilt_dir_deg = s.lastTiltDir;   // 무효 동안은 마지막 방향 유지 (크기는 0)

  // ---- 강풍 모드 ----
  float wmax = max(v, max(vA, g)); // 급격한 강풍은 평균창을 기다리지 않고 진입
  if (!s.safe) {
    if (wmax > WIND_SAFE_LIMIT_MS) { s.safe = true; s.safeReleasing = false; }
  } else if (wmax < WIND_SAFE_LIMIT_MS - WIND_SAFE_HYST_MS) {
    if (!s.safeReleasing) { s.safeReleasing = true; s.safeReleaseStartMs = nowMs; }
    else if (nowMs - s.safeReleaseStartMs >= SAFE_RELEASE_MS) { s.safe = false; s.safeReleasing = false; }
  } else {
    s.safeReleasing = false;
  }
  f.safe_mode = s.safe;

  // ---- 리브 목표각 ----
  // 강풍이거나, 센서 고장 + FAULT_TO_SAFE 이면 안전 자세. 고장 + FAULT_TO_NEUTRAL 이면 direction_valid=0 → 중립.
  bool poseSafe = f.safe_mode || (f.sensor_fault && FAULT_ACTION == FAULT_TO_SAFE);
  RibTargets target = mapRibTargets(f.tilt_dir_deg, f.tilt_deg_baseline,
                                    f.direction_valid, g_mode, poseSafe);
  g_ribCmd = g_limiter.update(target, SAMPLE_PERIOD_MS / 1000.0f);
  // 실제 모터 전송 시: JOINT_PRESENT[i][k] 인 관절만 jointDegToDxlTick(i, k, g_ribCmd.j[i][k]) 를 Goal Position 으로 보낸다.
}

// ============================================================================
// 6. 출력
// ============================================================================

void printHeader() {
  Serial.println(
    "t_ms,valid,vane_status,adc_mv,vane_r,dir_raw_deg,dir_rel_deg,"
    "speed_inst_ms,speed_win_ms,speed_avg_ms,gust_ms,u,w,dir_avg_deg,sin_dir,cos_dir,"
    "stability_R,direction_valid,protect_sector,sector_name,tilt_dir_deg,tilt_deg_baseline,"
    "mode,safe_mode,sensor_fault,err_events,err_samples,err_total_ms,err_cur_ms,err_max_ms,"
    "r1_j1,r1_j2,r2_j1,r2_j2,r3_j1,r3_j2,r4_j1,r4_j2,pose_safe");
}

void printCsv(const WindFeatures& f, const RibTargets& r) {
  char buf[512];
  int n = snprintf(buf, sizeof(buf),
    "%lu,%d,%s,%d,%.0f,%.1f,%.1f,"
    "%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.1f,%.4f,%.4f,"
    "%.3f,%d,%d,%s,%.1f,%.2f,"
    "%s,%d,%d,%lu,%lu,%lu,%lu,%lu",
    (unsigned long)f.t_ms, f.valid ? 1 : 0, VANE_STATUS_NAME[f.vane_status], f.adc_mv,
    f.vane_r_ohm, f.dir_from_raw_deg, f.dir_from_rel_deg,
    f.speed_inst_ms, f.speed_win_ms, f.speed_avg_ms, f.gust_ms, f.u_ms, f.w_ms,
    f.dir_from_avg_deg, f.sin_dir, f.cos_dir,
    f.stability_R, f.direction_valid ? 1 : 0, f.protect_sector,
    f.protect_sector >= 0 ? SECTOR_NAME[f.protect_sector] : "NONE",
    f.tilt_dir_deg, f.tilt_deg_baseline,
    MODE_NAME[g_mode], f.safe_mode ? 1 : 0, f.sensor_fault ? 1 : 0,
    (unsigned long)f.err_events, (unsigned long)f.err_samples, (unsigned long)f.err_total_ms,
    (unsigned long)f.err_cur_ms, (unsigned long)f.err_max_ms);
  for (int i = 0; i < NUM_RIBS; i++) {
    for (int k = 0; k < 2; k++) {
      if (JOINT_PRESENT[i][k]) n += snprintf(buf + n, sizeof(buf) - n, ",%.1f", r.j[i][k]);
      else                     n += snprintf(buf + n, sizeof(buf) - n, ",");   // 모터 없는 관절 = 빈 칸
    }
  }
  Serial.print(buf);
  Serial.printf(",%d\n", (f.safe_mode || (f.sensor_fault && FAULT_ACTION == FAULT_TO_SAFE)) ? 1 : 0);
}

// ============================================================================
// 7. 부팅 자가 테스트 ('#' 로 시작하는 줄은 Python 로거가 주석으로 무시)
// ============================================================================

void selfTest() {
  float m, R;
  { float d[2] = {350, 10}, wt[2] = {1, 1};
    circularMeanDeg(d, wt, 2, m, R);
    bool ok = fabsf(angDiff(m, 0)) < 0.5f && R > 0.95f;
    Serial.printf("# TEST circmean(350,10) = %.1f deg, R=%.3f -> %s\n", m, R, ok ? "PASS" : "FAIL"); }
  { float d[2] = {90, 270}, wt[2] = {5, 5};
    circularMeanDeg(d, wt, 2, m, R);
    bool ok = R < STABILITY_MIN_R;
    Serial.printf("# TEST circmean(90,270) R=%.3f -> direction_valid=0 %s\n", R, ok ? "PASS" : "FAIL"); }
  { int fail = 0;  // 테이블 저항 → 이론 전압 → 디코드가 같은 인덱스로 돌아오는지
    for (int i = 0; i < VANE_NUM_DIRS; i++) {
      int mv = (int)lroundf(VANE_VCC_MV * VANE_TABLE_OHM[i] / (VANE_TABLE_OHM[i] + VANE_R_PULLUP_OHM));
      VaneReading r = decodeVane(mv);
      if (r.status != VANE_OK || r.index != i) {
        fail++; Serial.printf("# TEST vane idx %d (%d mV) -> %s idx %d\n", i, mv, VANE_STATUS_NAME[r.status], r.index);
      }
    }
    Serial.printf("# TEST vane table decode -> %s\n", fail ? "FAIL" : "PASS"); }
  Serial.printf("# TEST decode(100mV)=%s, decode(142mV)=%s, decode(3250mV)=%s\n",
                VANE_STATUS_NAME[decodeVane(100).status], VANE_STATUS_NAME[decodeVane(142).status],
                VANE_STATUS_NAME[decodeVane(3250).status]);
  { bool ok = decodeVane(VANE_OPEN_MV).status == VANE_OPEN &&
              decodeVane((int)VANE_VCC_MV).status == VANE_OPEN;
    Serial.printf("# TEST ADC saturation guards -> %s\n", ok ? "PASS" : "FAIL"); }
  { bool ok = true;
    for (int k = 0; k < 16; k++) {
      int expected = ((k + 1) / 2) % 8;
      ok = ok && sectorFromDir(k * 22.5f, SECTOR_NONE) == expected;
    }
    ok = ok && sectorFromDir(27.0f, 0) == 0 && sectorFromDir(28.0f, 0) == 1 &&
         sectorFromDir(359.0f, 0) == 0;
    Serial.printf("# TEST 16-to-8 sectors / hysteresis -> %s\n", ok ? "PASS" : "FAIL"); }
  // 8방향 × 최대 기울기에서 관절 목표각 표 (Python selftest 표와 같아야 함)
  Serial.println("# TEST map8  dir  sector  r1_j1 r1_j2 r2_j1 r2_j2 r3_j1 r3_j2 r4_j1 r4_j2");
  for (int k = 0; k < 8; k++) {
    RibTargets t = mapRibTargets(k * 45.0f, TILT_MAX_DEG, true, MODE_WIDE, false);
    Serial.printf("# TEST map8 %4d  %-6s", k * 45, SECTOR_NAME[sectorFromDir(k * 45.0f, SECTOR_NONE)]);
    for (int i = 0; i < NUM_RIBS; i++) Serial.printf(" %5.1f %5.1f", t.j[i][0], t.j[i][1]);
    Serial.println();
  }
}

// ============================================================================
// 8. setup / loop (비차단, delay 없음)
// ============================================================================

uint32_t g_nextSampleMs = 0, g_nextOutputMs = 0;
bool g_btnLast = HIGH; uint32_t g_btnChangeMs = 0; bool g_btnStable = HIGH;

void pollButton(uint32_t nowMs) {
  if (!USE_MODE_BUTTON) return;
  bool b = digitalRead(PIN_MODE_BUTTON);
  if (b != g_btnLast) { g_btnLast = b; g_btnChangeMs = nowMs; }
  if (nowMs - g_btnChangeMs > 50 && b != g_btnStable) {   // 50 ms 디바운스
    g_btnStable = b;
    if (b == LOW) {                                        // 눌림 → 모드 토글
      g_mode = (CanopyMode)((g_mode + 1) % NUM_MODES);
      Serial.printf("# MODE -> %s\n", MODE_NAME[g_mode]);
    }
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'h') printHeader();
    else if (c == 'w') { g_mode = MODE_WIDE;   Serial.println("# MODE -> WIDE"); }
    else if (c == 'n') { g_mode = MODE_NARROW; Serial.println("# MODE -> NARROW"); }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_ANEMO, INPUT_PULLUP);
  if (USE_MODE_BUTTON) pinMode(PIN_MODE_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VANE, ADC_11db);  // 0~약 3.1 V 측정 범위
  attachInterrupt(digitalPinToInterrupt(PIN_ANEMO), onAnemoPulse, FALLING);

  Serial.println("# wind_preprocess start");
  Serial.printf("# AVG_N=%d GUST_N=%d EMA=%d MOUNT_OFFSET=%.1f\n", AVG_N, GUST_N, SMOOTH_USE_EMA, MOUNT_OFFSET_DEG);
  selfTest();
  printHeader();

  uint32_t now = millis();
  g_ws.winStartMs = now;
  g_nextSampleMs = now;
  g_nextOutputMs = now + OUTPUT_PERIOD_MS;
}

void loop() {
  uint32_t now = millis();
  pollSerial();
  pollButton(now);

  if ((int32_t)(now - g_nextSampleMs) >= 0) {
    g_nextSampleMs += SAMPLE_PERIOD_MS;
    if ((int32_t)(now - g_nextSampleMs) > (int32_t)SAMPLE_PERIOD_MS) g_nextSampleMs = now; // 크게 밀리면 재동기
    processSample(now);
  }
  if ((int32_t)(now - g_nextOutputMs) >= 0) {
    g_nextOutputMs += OUTPUT_PERIOD_MS;
    if ((int32_t)(now - g_nextOutputMs) > (int32_t)OUTPUT_PERIOD_MS) g_nextOutputMs = now;
    printCsv(g_feat, g_ribCmd);
  }
}
