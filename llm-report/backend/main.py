
import threading
import time

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from services.ros_adapter import convert_ros_data
from services.risk_rules import analyze_risk
from services.report_generator import generate_report

app = FastAPI()

# API 접근 허용 
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 더미데이터 (test용) : 추후에 ROS2 데이터로 변경 예정 
latest_sensor_data = {
    "gas": {
        "gas_type": "NH3",
        "concentration_ppm": 48.2,
        "alert_message": "가스 누출 감지",
    },
    "thermal": {
        "machines": [
            {
                "id": "machine_2",
                "temperature": 127.3,
            }
        ],
        "alert_message": "설비 과열 감지",
    },
    "location": "Factory Zone A",
}

latest_risk_result = analyze_risk(latest_sensor_data)

# ── 자동 리포트 (푸시 방식) ──────────────────────────────
# 위험 등급 전환 / 누출원 특정 순간 백그라운드에서 LLM 리포트를 자동 생성·보관.
# 프론트엔드는 GET /report/latest 를 폴링하다가 id가 바뀌면 알림으로 띄운다.
latest_report = {
    "id": 0,
    "created_at": None,
    "trigger": None,
    "report": None,
    "generating": False,
}
_report_lock = threading.Lock()
_prev_final_risk = "Normal"
_prev_source_found = False
_last_report_time = 0.0
REPORT_COOLDOWN_SEC = 60.0  # 같은 위험 상태 지속 시 재생성 최소 간격

# ── 위험도 래치 ──
# 추적/대피 중에는 로봇이 가스에서 멀어져 측정값이 떨어져도 경보를 내리지 않는다.
# 순찰(PATROL) 복귀 시에만 래치 해제 → 대시보드가 사고 종료 후에 Normal로 돌아옴.
RISK_RANK = {"Normal": 0, "Caution": 1, "Warning": 2, "Danger": 3}
_held_final_risk = "Normal"
_prev_source_key = None


def _generate_report_async(trigger: str):
    """LLM 호출은 수 초 걸리므로 백그라운드 스레드에서 생성"""
    with _report_lock:
        if latest_report["generating"]:
            return
        latest_report["generating"] = True

    def work():
        global _last_report_time
        try:
            report = generate_report(latest_sensor_data, latest_risk_result)
            with _report_lock:
                latest_report["id"] += 1
                latest_report["report"] = report
                latest_report["created_at"] = time.strftime("%H:%M:%S")
                latest_report["trigger"] = trigger
            _last_report_time = time.time()
        finally:
            with _report_lock:
                latest_report["generating"] = False

    threading.Thread(target=work, daemon=True).start()


# 더미데이터 -> 실제 데이터 교체
@app.post("/sensor")
def update_sensor(raw_data: dict):
    global latest_sensor_data
    global latest_risk_result
    global _prev_final_risk
    global _prev_source_found

    latest_sensor_data = convert_ros_data(raw_data)

    latest_risk_result = analyze_risk(
        latest_sensor_data
    )

    # ── 위험도 래치: 추적/대피 중에는 위험도가 내려가지 않게 유지 ──
    global _held_final_risk
    mode = latest_sensor_data.get("robot_mode")
    measured_risk = latest_risk_result.get("final_risk", "Normal")

    if mode in ("GAS_TRACKING", "EVACUATING"):
        if RISK_RANK.get(measured_risk, 0) > RISK_RANK.get(_held_final_risk, 0):
            _held_final_risk = measured_risk
        if RISK_RANK.get(_held_final_risk, 0) > RISK_RANK.get(measured_risk, 0):
            latest_risk_result["final_risk"] = _held_final_risk
            latest_risk_result["risk_held"] = True  # 대시보드 표시용
    else:
        _held_final_risk = "Normal"

    # ── 자동 리포트 트리거 판정 ──
    final_risk = latest_risk_result.get("final_risk", "Normal")
    src = latest_sensor_data.get("source_found")
    source_found = bool(src)

    # 누출원 좌표가 바뀌면 '다른 사건'으로 보고 새로 트리거
    # (백엔드 재시작 없이 반복 시연해도 매번 리포트가 나오도록)
    global _prev_source_key
    source_key = src.get("position_unity_xyz") if isinstance(src, dict) else None

    danger_now = final_risk in ("Warning", "Danger")
    became_danger = danger_now and _prev_final_risk not in ("Warning", "Danger")
    new_source = source_found and (
        not _prev_source_found or source_key != _prev_source_key
    )

    if new_source:
        # 누출원 특정 = 핵심 사건 → 쿨다운 무시하고 즉시 생성
        _generate_report_async("source_found")
    elif became_danger and time.time() - _last_report_time > REPORT_COOLDOWN_SEC:
        _generate_report_async(f"risk:{final_risk}")

    _prev_final_risk = final_risk
    _prev_source_found = source_found
    _prev_source_key = source_key

    return {
        "message": "sensor data updated",
        "sensor_data": latest_sensor_data,
        "risk": latest_risk_result,
    }


@app.get("/report/latest")
def get_latest_report():
    """저장된 최신 자동 리포트 (생성하지 않음 — 폴링용)"""
    with _report_lock:
        return dict(latest_report)

@app.get("/sensor")
def get_sensor():
    # 재계산하지 않고 래치가 반영된 최신 위험도를 그대로 반환
    return {
        "sensor_data": latest_sensor_data,
        "risk": latest_risk_result,
    }

@app.get("/report")
def get_report():

    report = generate_report(
        latest_sensor_data,
        latest_risk_result,
    )

    return {
        "sensor_data": latest_sensor_data,
        "risk": latest_risk_result,
        "report": report,

    }