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
    }
