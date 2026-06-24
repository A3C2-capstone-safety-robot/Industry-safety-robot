
# 센서값 입력 시 NH3 / H2S / CH4 기준 위험도 판단
# Normal / Caution / Warning / DAnger 반환
# 센서값 기준 1차 위험도 계산

# backend/services/risk_rules.py

GAS_THRESHOLDS = {
    "NH3": {
        "name": "ammonia",
        "caution": 25,
        "warning": 50,
        "danger": 300,
        "unit": "ppm",
        "description": "NIOSH REL 25ppm / OSHA PEL 50ppm / IDLH 300ppm",
    },
    "H2S": {
        "name": "hydrogen_sulfide",
        "caution": 10,
        "warning": 20,
        "danger": 100,
        "unit": "ppm",
        "description": "NIOSH Ceiling 10ppm / OSHA Ceiling 20ppm / IDLH 100ppm",
    },
    "CH4": {
        "name": "methane",
        "caution": 10,
        "warning": 20,
        "danger": 40,
        "unit": "%LEL",
        "description": "Methane lower explosive limit 기준",
    },
}

GAS_ALIASES = {
    "암모니아": "NH3",
    "ammonia": "NH3",
    "NH3": "NH3",
    "황화수소": "H2S",
    "hydrogen_sulfide": "H2S",
    "H2S": "H2S",
    "메탄": "CH4",
    "methane": "CH4",
    "CH4": "CH4",
}

TEMPERATURE_THRESHOLDS = {
    "caution": 80,
    "warning": 100,
    "danger": 120,
}

RISK_PRIORITY = {
    "Normal": 0,
    "Caution": 1,
    "Warning": 2,
    "Danger": 3,
}


def normalize_gas_type(gas_type: str) -> str:
    return GAS_ALIASES.get(gas_type, "NH3")


def classify_risk(value: float, thresholds: dict) -> str:
    if value >= thresholds["danger"]:
        return "Danger"
    if value >= thresholds["warning"]:
        return "Warning"
    if value >= thresholds["caution"]:
        return "Caution"
    return "Normal"


def get_higher_risk(risk_a: str, risk_b: str) -> str:
    if RISK_PRIORITY[risk_a] >= RISK_PRIORITY[risk_b]:
        return risk_a
    return risk_b


def analyze_risk(sensor_data: dict) -> dict:
    gas_data = sensor_data.get("gas", {})
    thermal_data = sensor_data.get("thermal", {})

    gas_type = normalize_gas_type(gas_data.get("gas_type", "NH3"))
    gas_value = float(gas_data.get("concentration_ppm", 0.0))

    gas_threshold = GAS_THRESHOLDS[gas_type]
    gas_risk = classify_risk(gas_value, gas_threshold)

    machines = thermal_data.get("machines", [])

    hottest_machine = None
    max_temperature = 0.0

    if machines:
        hottest_machine = max(
            machines,
            key=lambda machine: float(machine.get("temperature", 0.0)),
        )
        max_temperature = float(hottest_machine.get("temperature", 0.0))

    temperature_risk = classify_risk(max_temperature, TEMPERATURE_THRESHOLDS)
    final_risk = get_higher_risk(gas_risk, temperature_risk)

    return {
        "final_risk": final_risk,
        "gas_risk": gas_risk,
        "temperature_risk": temperature_risk,
        "gas_type": gas_type,
        "gas_value": gas_value,
        "gas_unit": gas_threshold["unit"],
        "gas_standard": gas_threshold["description"],
        "hottest_machine_id": hottest_machine.get("id") if hottest_machine else None,
        "max_temperature": max_temperature,
        "temperature_unit": "°C",
    }

