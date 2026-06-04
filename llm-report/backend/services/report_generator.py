# 센서값 + 검색 근거로 LLM 리포트 작성
# backend/services/report_generator.py

from services.llm import call_llm
from services.rag import retrieve_docs


def build_report_query(sensor_data: dict, risk_result: dict) -> str:
    gas_data = sensor_data.get("gas", {})
    thermal_data = sensor_data.get("thermal", {})

    gas_type = risk_result.get("gas_type", gas_data.get("gas_type", "UNKNOWN"))
    gas_value = risk_result.get("gas_value", gas_data.get("concentration_ppm", 0.0))
    gas_unit = risk_result.get("gas_unit", "ppm")

    final_risk = risk_result.get("final_risk", "Normal")
    gas_risk = risk_result.get("gas_risk", "Normal")
    temperature_risk = risk_result.get("temperature_risk", "Normal")

    hottest_machine_id = risk_result.get("hottest_machine_id")
    max_temperature = risk_result.get("max_temperature", 0.0)

    location = sensor_data.get("location", "Unknown")

    query_parts = [
        f"가스 종류 {gas_type}",
        f"가스 농도 {gas_value}{gas_unit}",
        f"가스 위험도 {gas_risk}",
        f"설비 온도 {max_temperature}도",
        f"온도 위험도 {temperature_risk}",
        f"최종 위험도 {final_risk}",
        f"위치 {location}",
    ]

    if hottest_machine_id:
        query_parts.append(f"과열 설비 {hottest_machine_id}")

    return " ".join(query_parts)


def build_context(retrieved_docs: list[dict]) -> str:
    if not retrieved_docs:
        return "검색된 참고 문서가 없습니다."

    return "\n\n".join(
        [
            f"[출처: {doc.get('source', 'unknown')}]\n{doc.get('content', '')}"
            for doc in retrieved_docs
        ]
    )


def format_status_events(sensor_data: dict) -> str:
    """로봇이 발행한 시간순 상황 보고 → 리포트의 '사고 경위' 재료"""
    events = sensor_data.get("status_events") or []
    if not events:
        return "이벤트 기록 없음"

    lines = []
    for event in events[-15:]:  # 최근 15개만 (프롬프트 길이 관리)
        lines.append(f"- {event.get('time', '?')} {event.get('text', '')}")
    return "\n".join(lines)


def format_source_found(sensor_data: dict) -> str:
    """누출원 특정 결과 요약"""
    src = sensor_data.get("source_found")
    if not src:
        return "누출원 미특정 (탐색 전이거나 진행 중)"

    danger = "위험 — 대피 필요" if src.get("danger") else "주의 수준"
    return (
        f"가스 {src.get('gas_type', '?')} | "
        f"최고 농도 {src.get('peak_concentration', '?')} ppm | "
        f"위치 좌표 {src.get('position_unity_xyz', '?')} | 판정: {danger}"
    )


def format_machine_status(sensor_data: dict) -> str:
    machines = sensor_data.get("thermal", {}).get("machines", [])

    if not machines:
        return "설비 온도 데이터 없음"

    lines = []
    for machine in machines:
        machine_id = machine.get("id", "unknown")
        temperature = machine.get("temperature", 0.0)
        lines.append(f"- {machine_id}: {temperature}°C")

    return "\n".join(lines)

def generate_report(sensor_data: dict, risk_result: dict) -> str:
    query = build_report_query(sensor_data, risk_result)
    retrieved_docs = retrieve_docs(query, top_k=3)
    context = build_context(retrieved_docs)

    gas_data = sensor_data.get("gas", {})
    thermal_data = sensor_data.get("thermal", {})

    prompt = f"""
너는 산업안전 사고 대응 전문가다.
아래 센서 데이터, 위험도 계산 결과, 참고 문서를 근거로 공장 내 사고 상황 리포트를 한국어로 작성하라.

[센서 데이터]
가스 종류: {risk_result.get("gas_type", gas_data.get("gas_type", "UNKNOWN"))}
가스 농도: {risk_result.get("gas_value", gas_data.get("concentration_ppm", 0.0))} {risk_result.get("gas_unit", "ppm")}
위치: {sensor_data.get("location", "Unknown")}

설비 온도 목록:
{format_machine_status(sensor_data)}

가스 경보 메시지: {gas_data.get("alert_message", "없음")}
과열 경보 메시지: {thermal_data.get("alert_message", "없음")}

[로봇 대응 기록]
현재 로봇 모드: {sensor_data.get("robot_mode") or "알 수 없음"} (PATROL=순찰, GAS_TRACKING=누출원 추적, EVACUATING=대피)
누출원 특정 결과: {format_source_found(sensor_data)}

사건 타임라인 (시간순 로봇 보고):
{format_status_events(sensor_data)}

[위험도 계산 결과]
최종 위험도: {risk_result.get("final_risk", "Normal")}
가스 위험도: {risk_result.get("gas_risk", "Normal")}
온도 위험도: {risk_result.get("temperature_risk", "Normal")}
최고 온도 설비: {risk_result.get("hottest_machine_id", "없음")}
최고 온도: {risk_result.get("max_temperature", 0.0)}°C
가스 기준: {risk_result.get("gas_standard", "기준 정보 없음")}

[참고 문서]
{context}

[작성 형식]
1. 상황 요약
2. 사고 경위 (사건 타임라인 기반, 시간 명시)
3. 위험 수준
4. 추정 원인
5. 누출원 위치 및 로봇 대응 현황
6. 즉시 조치
7. 대피 지침
8. 참고 근거

[주의사항]
- 참고 문서에 없는 내용을 단정하지 말 것
- 원인은 “가능성”으로 표현할 것
- 센서 수치와 위험도 계산 결과를 반드시 반영할 것
- 작업자가 바로 이해할 수 있게 작성할 것
- 너무 길게 쓰지 말 것
"""

    return call_llm(prompt)