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

    # ── 자동 리포트 트리거 판정 ──
    final_risk = latest_risk_result.get("final_risk", "Normal")
    source_found = bool(latest_sensor_data.get("source_found"))

    danger_now = final_risk in ("Warning", "Danger")
    became_danger = danger_now and _prev_final_risk not in ("Warning", "Danger")
    new_source = source_found and not _prev_source_found

    if new_source:
        # 누출원 특정 = 핵심 사건 → 쿨다운 무시하고 즉시 생성
        _generate_report_async("source_found")
    elif became_danger and time.time() - _last_report_time > REPORT_COOLDOWN_SEC:
        _generate_report_async(f"risk:{final_risk}")

    _prev_final_risk = final_risk
    _prev_source_found = source_found

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
    risk_result = analyze_risk(latest_sensor_data)

    return {
        "sensor_data": latest_sensor_data,
        "risk": risk_result,
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