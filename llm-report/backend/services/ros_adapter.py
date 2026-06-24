def convert_ros_data(raw_data: dict) -> dict:
    """
    ROS 토픽에서 넘어온 raw 데이터를
    백엔드 내부 표준 sensor_data 형식으로 변환한다.
    """

    gas_value = raw_data.get("gas_concentration", raw_data.get("gas_value", 0.0))
    gas_alert = raw_data.get("gas_alert", raw_data.get("alert", None))

    temperatures = raw_data.get(
        "machine_temperatures",
        raw_data.get("temperatures", []),
    )

    thermal_alert = raw_data.get("thermal_alert", None)

    machines = []

    # ── 우선순위 1: 로봇 열화상 실측값 (화각+가림 필터 통과, 마지막 관측 유지) ──
    # measured_machines 키가 존재하면 (빈 리스트여도) 실측 모드로 동작:
    # 아직 아무것도 관측 못 했으면 machines가 비고, 온도 위험도는 Normal.
    if "measured_machines" in raw_data:
        for m in raw_data.get("measured_machines") or []:
            machines.append(
                {
                    "id": m.get("id", "unknown"),
                    "temperature": float(m.get("temperature", 0.0)),
                    "last_seen": m.get("last_seen"),
                }
            )
    else:
        # ── 우선순위 2 (구버전 호환): 시뮬레이터 정답지 온도 ──
        for idx, temp in enumerate(temperatures):
            machines.append(
                {
                    "id": f"machine_{idx + 1}",
                    "temperature": float(temp),
                }
            )

        if not machines and "temperature" in raw_data:
            machines.append(
                {
                    "id": raw_data.get("machine_id", "machine_1"),
                    "temperature": float(raw_data["temperature"]),
                }
            )

    return {
        "gas": {
            "gas_type": raw_data.get("gas_type", "UNKNOWN"),
            "concentration_ppm": float(gas_value),
            "alert_message": gas_alert,
        },
        "thermal": {
            "machines": machines,
            "alert_message": thermal_alert,
        },
        "location": raw_data.get("location", "Unknown"),
        # ── 사건 데이터 (llm_bridge가 발행) — 리포트의 경위/대응 섹션 재료 ──
        "robot_mode": raw_data.get("robot_mode"),          # PATROL / GAS_TRACKING / EVACUATING
        "source_found": raw_data.get("source_found"),      # 누출원 특정 결과 (좌표·농도·위험)
        "status_events": raw_data.get("status_events", []),# 시간순 상황 보고 타임라인
    }
