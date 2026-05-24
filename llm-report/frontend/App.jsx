import { useEffect, useState } from "react";

function App() {
  const [sensor, setSensor] = useState(null);
  const [risk, setRisk] = useState(null);
  const [report, setReport] = useState("");
  const [error, setError] = useState("");

  useEffect(() => {
    async function fetchDashboardData() {
      try {
        const sensorRes = await fetch("http://127.0.0.1:8000/sensor");
        const sensorJson = await sensorRes.json();

        setSensor(sensorJson.sensor_data);
        setRisk(sensorJson.risk);

        const reportRes = await fetch("http://127.0.0.1:8000/report");
        const reportJson = await reportRes.json();

        setReport(reportJson.report);
      } catch (err) {
        console.error(err);
        setError("데이터를 불러오지 못했습니다.");
      }
    }

    fetchDashboardData();
  }, []);

  if (error) {
    return <div>{error}</div>;
  }

  if (!sensor || !risk) {
    return <div>Loading...</div>;
  }

  const gas = sensor.gas;
  const thermal = sensor.thermal;
  const hottestMachine = thermal.machines?.[0];

  return (
    <div style={{ padding: "30px", fontFamily: "Arial" }}>
      <h1>Factory AI Dashboard</h1>

      <div style={{ border: "1px solid gray", padding: "20px", marginBottom: "20px", borderRadius: "10px" }}>
        <h2>위험 수준</h2>
        <h1 style={{ color: "red" }}>{risk.final_risk}</h1>
      </div>

      <div style={{ border: "1px solid gray", padding: "20px", marginBottom: "20px", borderRadius: "10px" }}>
        <h2>센서 데이터</h2>
        <p>가스 종류: {gas.gas_type}</p>
        <p>가스 농도: {gas.concentration_ppm} ppm</p>
        <p>기계 ID: {hottestMachine?.id}</p>
        <p>온도: {hottestMachine?.temperature} °C</p>
        <p>위치: {sensor.location}</p>
        <p>가스 경보: {gas.alert_message}</p>
        <p>과열 경보: {thermal.alert_message}</p>
      </div>

      <div style={{ border: "1px solid gray", padding: "20px", borderRadius: "10px", whiteSpace: "pre-line" }}>
        <h2>LLM 리포트</h2>
        {report || "리포트 생성 중..."}
      </div>
    </div>
  );
}

export default App;