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
    retrieved_docs = retrieve_docs(query, top_k=5)
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
2. 위험 수준
3. 추정 원인
4. 즉시 조치
5. 대피 지침
6. 참고 근거

[주의사항]
- 참고 문서에 없는 내용을 단정하지 말 것
- 원인은 “가능성”으로 표현할 것
- 센서 수치와 위험도 계산 결과를 반드시 반영할 것
- 작업자가 바로 이해할 수 있게 작성할 것
- 너무 길게 쓰지 말 것
"""

    return call_llm(prompt)
