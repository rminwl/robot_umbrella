"""
wind_logger.py — 풍향·풍속 전처리 데이터 PC 쪽 수집/분석 스크립트
(캡스톤 "환경적응형 웨어러블 로봇 우산")

ESP32(wind_preprocess.ino)가 5 Hz로 보내는 CSV를 받아 저장하고,
pandas로 정리·시각화·학습용 특징 행렬을 만든다.

각도 규약 (펌웨어와 동일)
  - 풍향 = 바람이 "불어오는(FROM)" 방향
  - 사용자 정면 0°, 위에서 봤을 때 시계방향 증가
  - u = v·sinθ (+: 오른쪽에서), w = v·cosθ (+: 정면에서)

사용법
  pip install pyserial pandas numpy matplotlib
  python wind_logger.py log --port COM5                 # 수집 (Ctrl+C로 종료)
  python wind_logger.py log --port COM5 --duration 300  # 300초만 수집
  python wind_logger.py analyze wind_20261116_153000.csv
  python wind_logger.py selftest                        # 원형 평균·매핑 검증
"""

import argparse
import csv
import datetime as dt
import math
import os
import sys
import time

import numpy as np
import pandas as pd

# ============================================================================
# 1. 파라미터 (펌웨어 상단 상수와 반드시 같게 유지)
# ============================================================================

BAUD = 115200                  # 시리얼 속도 [bps]
OUT_DIR = "logs"               # CSV 저장 폴더
LABEL_COLUMNS = ["target_tilt_dir_deg", "target_tilt_deg"]  # 학습 라벨 (수집 시 빈 칸)

# 학습 입력 특징 (순서 고정)
FEATURE_COLUMNS = ["sin_dir", "cos_dir", "speed_avg_ms", "gust_ms", "stability_R"]

STABILITY_MIN_R = 0.30         # 방향 유효 최소 안정도 (펌웨어와 동일)
TILT_MAX_DEG = 30.0            # 최대 기울기 [deg]

# 리브 / 관절 (펌웨어 8절 파라미터와 동일)
# r1=Top_L, r2=Top_R, r3=Side_L, r4=Side_R (등에 메는 하우징, 등 뒤를 막는 리브 없음)
RIB_NAME = ["Top_L", "Top_R", "Side_L", "Side_R"]
RIB_ANGLE_DEG = [315.0, 45.0, 270.0, 90.0]  # 리브가 덮는 방향 [deg] — 설계 이미지 기준 추정값

# [수정 가능성 있음] 모터 수 미확정 (예상 6~8개). 모터가 달린 관절만 True.
#   모터 없는 관절은 NaN 으로 반환 (펌웨어 CSV 에서는 빈 칸)
#   예) 6개: [[True, True], [True, True], [True, False], [True, False]]
# [수정 가능성 있음] 옆리브 J1 이 수평 회전일 수 있고, 윗리브 끝은 고정 굽힘일 수 있다.
JOINT_PRESENT = [[True, True], [True, True], [True, True], [True, True]]
J1_GAIN_DOWN = 25.0            # 바람 쪽 J1 최대 내림 [deg]
J2_GAIN_DOWN = 30.0            # 바람 쪽 J2 최대 굽힘 [deg]
J1_GAIN_UP = 10.0              # 반대쪽 J1 최대 올림 [deg]
J1_MIN, J1_MAX = -45.0, 30.0   # J1 범위 [deg]
J2_MIN, J2_MAX = -60.0, 10.0   # J2 범위 [deg]
J1_SAFE, J2_SAFE = -35.0, -45.0  # 강풍 안전 자세 [deg]
NEUTRAL_POSE = {               # 캐노피 면적 모드별 중립 자세 (J1, J2) [deg]
    "WIDE": (0.0, 0.0),
    "NARROW": (-15.0, -20.0),
}

# 다이나믹셀 위치 변환
# [수정 가능성 있음] 기종 미확정: XL330-M288-T 또는 XL430-W250-T.
#   둘 다 0~4095, 0.088°/단위라 변환은 같다. 전원·무게·토크는 기종 확정 후 다시 확인.
DXL_DEG_PER_TICK = 0.088       # Goal Position 1 단위 [deg]
JOINT_ZERO_TICK = [[2048, 2048]] * 4  # 관절각 0°의 tick (리브별 J1, J2)
JOINT_DIR_SIGN = [[1, 1]] * 4         # 관절 + 방향이 tick 증가면 1, 아니면 -1

JOINT_COLUMNS = [f"r{i + 1}_j{k + 1}" for i in range(4) for k in range(2)]

# ============================================================================
# 2. 수집 (pyserial)
# ============================================================================


def log_serial(port, baud=BAUD, out_dir=OUT_DIR, duration_s=None):
    """ESP32 CSV를 받아 날짜·시간 파일명으로 저장. 라벨 빈 컬럼을 덧붙인다."""
    import serial  # 분석만 할 때는 pyserial 없이도 동작하도록 여기서 import

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, dt.datetime.now().strftime("wind_%Y%m%d_%H%M%S.csv"))

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.5)            # 포트 열 때 ESP32가 리셋될 수 있으니 잠깐 대기
    ser.reset_input_buffer()
    ser.write(b"h")            # 헤더 재요청 (리셋되지 않은 경우 대비)

    header = None
    n_rows = 0
    t0 = time.time()
    print(f"[log] {port} → {path}  (Ctrl+C로 종료)")
    try:
        with open(path, "w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            while duration_s is None or time.time() - t0 < duration_s:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue
                if raw.startswith("#"):            # 펌웨어 메시지/자가 테스트
                    print(raw)
                    continue
                fields = raw.split(",")
                if fields[0] == "t_ms":
                    if header is None:
                        header = fields
                        writer.writerow(header + LABEL_COLUMNS)
                    continue                         # 중복 헤더는 무시
                if header is None or len(fields) != len(header):
                    continue                         # 헤더 전 데이터, 깨진 줄
                writer.writerow(fields + [""] * len(LABEL_COLUMNS))
                n_rows += 1
                if n_rows % 25 == 0:                 # 5초마다 상태 표시
                    fp.flush()
                    row = dict(zip(header, fields))
                    print(f"  t={row['t_ms']}ms v={row['speed_avg_ms']} dir={row['dir_avg_deg']} "
                          f"R={row['stability_R']} sector={row['sector_name']} tilt={row['tilt_deg_baseline']}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print(f"[log] {n_rows}행 저장: {path}")
    return path

# ============================================================================
# 3. 분석 (pandas)
# ============================================================================


def load_csv(path):
    return pd.read_csv(path)


def error_report(df):
    """센서 오류 통계. 펌웨어 누적 카운터(10 Hz 기준)를 우선 쓰고, 없으면 CSV 행(5 Hz)으로 계산한다."""
    print("---- 센서 오류 ----")
    if "err_events" in df and len(df):
        last = df.iloc[-1]
        span_ms = max(int(df["t_ms"].iloc[-1] - df["t_ms"].iloc[0]), 1)
        print(f"오류 발생 {int(last['err_events'])}회, 누적 {int(last['err_total_ms']) / 1000:.1f} s "
              f"(기록 시간의 {int(last['err_total_ms']) / span_ms:.1%}), 최장 {int(last['err_max_ms']) / 1000:.1f} s")
        if "sensor_fault" in df:
            print(f"센서 고장(추적 중단) 상태 비율: {df['sensor_fault'].mean():.1%}")
        # ESP32가 재부팅되면 카운터가 0으로 돌아가므로 감소 지점을 알려준다
        resets = int((df["err_events"].diff() < 0).sum())
        if resets:
            print(f"※ 카운터 초기화 {resets}회 감지 (ESP32 재부팅) — 위 값은 마지막 부팅 이후 기준")
    # CSV 행 기준 구간 목록 (5 Hz라 0.2 s 미만 오류는 빠질 수 있음)
    bad = df["valid"] != 1
    seg_id = (bad != bad.shift()).cumsum()
    segs = [(g["t_ms"].iloc[0], g["t_ms"].iloc[-1], g["vane_status"].mode().iloc[0])
            for _, g in df[bad].groupby(seg_id[bad])]
    print(f"CSV상 오류 구간 {len(segs)}개", "(상태별 행 수: " + str(df.loc[bad, "vane_status"].value_counts().to_dict()) + ")")
    for t0, t1, st in segs[:20]:
        print(f"  {t0 / 1000:8.1f} ~ {t1 / 1000:8.1f} s  {st}")
    if len(segs) > 20:
        print(f"  ... 외 {len(segs) - 20}개")
    return segs


def clean(df, verbose=True):
    """valid=0 행을 제거하고 통계를 출력한다. (오류 통계는 제거 전에 먼저 출력)"""
    n = len(df)
    if verbose:
        error_report(df)
    out = df[df["valid"] == 1].copy()
    if verbose:
        print(f"전체 {n}행, valid=1 {len(out)}행, 제거 {n - len(out)}행 ({(n - len(out)) / max(n, 1):.1%})")
        if "vane_status" in df:
            print("vane_status 분포:", df["vane_status"].value_counts().to_dict())
        print(f"direction_valid=1 비율: {out['direction_valid'].mean():.1%}")
        print(f"safe_mode=1 비율: {out['safe_mode'].mean():.1%}")
        if len(out):
            print(f"기록 시간: {(out['t_ms'].iloc[-1] - out['t_ms'].iloc[0]) / 1000:.1f} s")
            print(out[["speed_inst_ms", "speed_avg_ms", "gust_ms", "stability_R",
                       "tilt_deg_baseline"]].describe().round(3).to_string())
            print("보호 섹터 분포:", out["sector_name"].value_counts().to_dict())
    return out


def plot_direction_hist(df, col="dir_rel_deg", ax=None):
    """16방향 풍향 분포 (극좌표, 정면 0° 위쪽, 시계방향)."""
    import matplotlib.pyplot as plt

    idx = np.round(df[col].to_numpy() / 22.5).astype(int) % 16
    counts = np.bincount(idx, minlength=16)
    if ax is None:
        _, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(5, 5))
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.bar(np.deg2rad(np.arange(16) * 22.5), counts, width=np.deg2rad(20), alpha=0.8)
    ax.set_title(f"풍향(FROM) 분포 — {col}")
    return ax


def plot_speed(df, ax=None):
    """풍속 시계열: 순간 / 평활 / 돌풍, 무풍 구간 음영."""
    import matplotlib.pyplot as plt

    if ax is None:
        _, ax = plt.subplots(figsize=(10, 4))
    t = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    ax.plot(t, df["speed_inst_ms"], lw=0.7, alpha=0.5, label="순간")
    ax.plot(t, df["speed_avg_ms"], lw=1.6, label="평균")
    ax.plot(t, df["gust_ms"], lw=1.0, ls="--", label="돌풍")
    ax.fill_between(t, 0, 1, where=df["direction_valid"] == 0,
                    transform=ax.get_xaxis_transform(), color="gray", alpha=0.15, label="방향 무효")
    ax.set_xlabel("시간 [s]")
    ax.set_ylabel("풍속 [m/s]")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    return ax


def build_features(df, require_direction_valid=False):
    """학습용 X = [sin_dir, cos_dir, speed_avg_ms, gust_ms, stability_R].

    라벨이 채워진 행이 있으면 y = [target_tilt_dir_deg, target_tilt_deg] 도 반환한다.
    (라벨 각도는 회귀 시 sin/cos 로 바꿔 학습하는 것을 권장 → label_to_sincos)
    """
    d = df[df["valid"] == 1]
    if require_direction_valid:
        d = d[d["direction_valid"] == 1]
    X = d[FEATURE_COLUMNS].to_numpy(dtype=float)
    y = None
    if all(c in d for c in LABEL_COLUMNS):
        lab = d[LABEL_COLUMNS].dropna()
        if len(lab):
            X = d.loc[lab.index, FEATURE_COLUMNS].to_numpy(dtype=float)
            y = lab.to_numpy(dtype=float)
    return X, y


def label_to_sincos(y):
    """[dir_deg, tilt_deg] → [sin, cos, tilt_deg]. 0°/360° 불연속을 없앤다."""
    r = np.deg2rad(y[:, 0])
    return np.column_stack([np.sin(r), np.cos(r), y[:, 1]])


def sincos_to_label(p):
    """모델 출력 [sin, cos, tilt_deg] → [dir_deg, tilt_deg]."""
    return np.column_stack([np.rad2deg(np.arctan2(p[:, 0], p[:, 1])) % 360, p[:, 2]])

# ============================================================================
# 4. 원형 평균 / 리브 관절 매핑 (펌웨어와 같은 순수 함수)
# ============================================================================


def circular_mean_deg(deg, weight=None):
    """가중 원형 평균 → (평균 각도, R). 가중치 합 0 이면 (nan, 0)."""
    deg = np.asarray(deg, dtype=float)
    w = np.ones_like(deg) if weight is None else np.asarray(weight, dtype=float)
    su = np.sum(w * np.sin(np.deg2rad(deg)))
    sw = np.sum(w * np.cos(np.deg2rad(deg)))
    sv = np.sum(w)
    if sv <= 1e-6:
        return float("nan"), 0.0
    mean = round(float(np.rad2deg(np.arctan2(su, sw))), 6) % 360  # -1e-15 → 360.0 방지
    return mean, float(math.hypot(su, sw) / sv)


def map_rib_targets(tilt_dir_deg, tilt_deg, direction_valid=True, mode="WIDE", safe_mode=False):
    """(방향, 크기, 모드) → 8관절 목표각 dict {r1_j1: .., ..., r4_j2: ..} [deg].

    학습 모델 출력(target_tilt_dir_deg, target_tilt_deg)을 그대로 넣어 쓴다.
    (데드밴드·속도 제한은 상태가 필요하므로 펌웨어 JointLimiter 에서 처리)
    """
    n1, n2 = NEUTRAL_POSE[mode]
    s = min(max(tilt_deg / TILT_MAX_DEG, 0.0), 1.0)
    out = {}
    for i, rib in enumerate(RIB_ANGLE_DEG):
        if safe_mode:
            j1, j2 = J1_SAFE, J2_SAFE
        elif not direction_valid:
            j1, j2 = n1, n2
        else:
            c = math.cos(math.radians(rib - tilt_dir_deg))
            if c > 0:
                j1 = n1 - J1_GAIN_DOWN * c * s
                j2 = n2 - J2_GAIN_DOWN * c * s
            else:
                j1 = n1 + J1_GAIN_UP * (-c) * s
                j2 = n2
        j1 = min(max(j1, J1_MIN), J1_MAX)
        j2 = min(max(j2, J2_MIN), J2_MAX)
        out[f"r{i + 1}_j1"] = j1 if JOINT_PRESENT[i][0] else float("nan")
        out[f"r{i + 1}_j2"] = j2 if JOINT_PRESENT[i][1] else float("nan")
    return out


def deg_to_dxl_tick(rib, joint, deg):
    """관절각 [deg] → 다이나믹셀 Goal Position [0~4095] (XL330/XL430 공통). rib: 0~3, joint: 0(J1)/1(J2)."""
    tick = JOINT_ZERO_TICK[rib][joint] + JOINT_DIR_SIGN[rib][joint] * deg / DXL_DEG_PER_TICK
    return int(min(max(round(tick), 0), 4095))


def add_joint_columns(df, dir_col="target_tilt_dir_deg", tilt_col="target_tilt_deg", prefix="pred_"):
    """df 각 행의 (방향, 크기)로 8관절 목표각 컬럼을 계산해 붙인다."""
    rows = [map_rib_targets(r[dir_col], r[tilt_col], bool(r["direction_valid"]),
                            r.get("mode", "WIDE"), bool(r.get("safe_mode", 0)))
            for _, r in df.iterrows()]
    joints = pd.DataFrame(rows, index=df.index).add_prefix(prefix)
    return pd.concat([df, joints], axis=1)

# ============================================================================
# 5. 자가 테스트
# ============================================================================


def run_selftest():
    ok = True

    m, R = circular_mean_deg([350, 10])
    p = abs(((m + 180) % 360) - 180) < 0.5 and R > 0.95
    print(f"circmean(350,10) = {m:.1f}°, R={R:.3f}  (산술평균이면 180°) → {'PASS' if p else 'FAIL'}")
    ok &= p

    m, R = circular_mean_deg([90, 270], [5, 5])
    p = R < STABILITY_MIN_R
    print(f"circmean(90,270, 같은 풍속) R={R:.3f} → direction_valid=0 {'PASS' if p else 'FAIL'}")
    ok &= p

    def j1(t, rib):  # 모터 없는 관절(NaN)은 비교에서 통과로 본다
        v = t[f"r{rib}_j1"]
        return None if math.isnan(v) else v

    def show(t):
        return {k: (None if math.isnan(v) else round(v, 1)) for k, v in t.items()}

    # 정면(0°): 윗리브 Top_L(315°)·Top_R(45°)는 내림, 옆리브(270°, 90°)는 c=0 → 중립
    t = map_rib_targets(0.0, TILT_MAX_DEG)
    p = j1(t, 1) < 0 and j1(t, 2) < 0 and abs(j1(t, 3)) < 1e-6 and abs(j1(t, 4)) < 1e-6
    print("map(정면 0°, 30°):", show(t), "→", "PASS" if p else "FAIL")
    ok &= p

    # 오른쪽(90°): Side_R 가장 많이 내림, Side_L 올림
    t = map_rib_targets(90.0, TILT_MAX_DEG)
    p = j1(t, 4) < j1(t, 2) < 0 and j1(t, 3) > 0
    print("map(오른쪽 90°, 30°):", show(t), "→", "PASS" if p else "FAIL")
    ok &= p

    # 등 뒤(180°): 바람 쪽으로 내리는 리브 없음 (막는 리브가 없는 방향)
    t = map_rib_targets(180.0, TILT_MAX_DEG)
    p = all(v >= -1e-6 for v in t.values() if not math.isnan(v))
    print("map(등 뒤 180°, 30°):", show(t), "→", "PASS" if p else "FAIL")
    ok &= p

    # 8방향 전체: 표 출력 + 규칙 검사
    #   (a) 가장 정렬된 리브(c 최대, c>0)가 J1을 가장 많이 내림
    #   (b) c<=0 인 리브는 J2 = 중립 (굽히지 않음)
    #   (c) 좌우 대칭: 방향 d 와 360-d 에서 L/R 리브 값이 서로 바뀜
    sect = ["F", "FR", "R", "BR", "B", "BL", "L", "FL"]
    print("8방향 (최대 기울기, WIDE)   " + " ".join(f"{c:>6}" for c in JOINT_COLUMNS))
    p8 = True
    mirror = {1: 2, 2: 1, 3: 4, 4: 3}
    for k in range(8):
        d = k * 45.0
        t = map_rib_targets(d, TILT_MAX_DEG)
        print(f"  {d:5.0f}° {sect[k]:>2}               " +
              " ".join("   -  " if math.isnan(v) else f"{v:6.1f}" for v in t.values()))
        c = [math.cos(math.radians(a - d)) for a in RIB_ANGLE_DEG]
        j1s = [t[f"r{i + 1}_j1"] for i in range(4)]
        if max(c) > 1e-6 and not any(math.isnan(x) for x in j1s):
            p8 &= j1s.index(min(j1s)) == c.index(max(c)) or abs(min(j1s) - j1s[c.index(max(c))]) < 1e-6
        for i in range(4):
            v = t[f"r{i + 1}_j2"]
            if c[i] <= 1e-6 and not math.isnan(v):
                p8 &= abs(v - NEUTRAL_POSE["WIDE"][1]) < 1e-6  # cos(90°)≈6e-17 오차 허용
        tm = map_rib_targets((360 - d) % 360, TILT_MAX_DEG)
        for i in range(1, 5):
            for jj in (1, 2):
                a, b = t[f"r{i}_j{jj}"], tm[f"r{mirror[i]}_j{jj}"]
                if not (math.isnan(a) or math.isnan(b)):
                    p8 &= abs(a - b) < 1e-6
    print("8방향 규칙(최대 정렬 리브 최대 하강 / 반대쪽 J2 중립 / 좌우 대칭) →", "PASS" if p8 else "FAIL")
    ok &= p8

    vals = list(map_rib_targets(90, 20, False).items())
    p = all(math.isnan(v) or v == NEUTRAL_POSE["WIDE"][int(k[-1]) - 1] for k, v in vals)
    print("map(무풍) → 전부 중립", "PASS" if p else "FAIL")
    ok &= p

    vals = list(map_rib_targets(90, 20, True, safe_mode=True).items())
    p = all(math.isnan(v) or v == (J1_SAFE, J2_SAFE)[int(k[-1]) - 1] for k, v in vals)
    print("map(강풍) → 전부 안전 자세", "PASS" if p else "FAIL")
    ok &= p

    p = deg_to_dxl_tick(0, 0, 0) == 2048 and deg_to_dxl_tick(0, 0, 90) == 2048 + round(90 / 0.088)
    print(f"deg_to_dxl_tick(0°)={deg_to_dxl_tick(0, 0, 0)}, (90°)={deg_to_dxl_tick(0, 0, 90)} →",
          "PASS" if p else "FAIL")
    ok &= p

    print("전체:", "PASS" if ok else "FAIL")
    return ok

# ============================================================================
# 6. CLI
# ============================================================================


def main():
    ap = argparse.ArgumentParser(description="풍향·풍속 로거/분석기")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("log", help="ESP32 시리얼 → CSV 저장")
    p.add_argument("--port", required=True, help="예: COM5, /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--out", default=OUT_DIR)
    p.add_argument("--duration", type=float, default=None, help="수집 시간 [s]")

    p = sub.add_parser("analyze", help="저장된 CSV 통계·그래프")
    p.add_argument("csv")
    p.add_argument("--no-plot", action="store_true")

    sub.add_parser("selftest", help="원형 평균·매핑 검증")

    a = ap.parse_args()
    if a.cmd == "log":
        log_serial(a.port, a.baud, a.out, a.duration)
    elif a.cmd == "analyze":
        df = clean(load_csv(a.csv))
        X, y = build_features(df)
        print(f"X shape = {X.shape}, y = {None if y is None else y.shape}")
        if not a.no_plot:
            import matplotlib.pyplot as plt
            plt.rcParams["font.family"] = "Malgun Gothic" if sys.platform == "win32" else plt.rcParams["font.family"]
            plt.rcParams["axes.unicode_minus"] = False
            fig = plt.figure(figsize=(14, 5))
            plot_direction_hist(df, ax=fig.add_subplot(1, 3, 1, projection="polar"))
            plot_speed(df, ax=fig.add_subplot(1, 3, (2, 3)))
            fig.tight_layout()
            plt.show()
    elif a.cmd == "selftest":
        sys.exit(0 if run_selftest() else 1)


if __name__ == "__main__":
    main()
