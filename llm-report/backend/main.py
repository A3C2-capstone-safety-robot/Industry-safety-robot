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

# 더미데이터 -> 실제 데이터 교체 
@app.post("/sensor")
def update_sensor(raw_data: dict):
    global latest_sensor_data
    global latest_risk_result

    latest_sensor_data = convert_ros_data(raw_data)

    latest_risk_result = analyze_risk(
        latest_sensor_data
    )

    return {
        "message": "sensor data updated",
        "sensor_data": latest_sensor_data,
        "risk": latest_risk_result,
    }

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