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
    return <div style={styles.center}>{error}</div>;
  }

  if (!sensor || !risk) {
    return <div style={styles.center}>Loading...</div>;
  }

  const gas = sensor.gas;
  const thermal = sensor.thermal;
  const hottestMachine = thermal.machines?.[0];
  const riskColor = {
    Normal: "#16a34a",
    Caution: "#eab308",
    Warning: "#f97316",
    Danger: "#dc2626",
  }[risk.final_risk] || "#6b7280";

  const summary = `${gas.gas_type} ${gas.concentration_ppm}ppm 감지 및 ${hottestMachine?.id} 과열 감지. ${sensor.location} 접근 통제 및 즉시 대응이 필요합니다.`;
  
   return (
    <div style={styles.page}>
      <div style={styles.header}>
        <div>
          <p style={styles.label}>AI Industrial Safety Monitoring</p>
          <h1 style={styles.title}>Factory AI Dashboard</h1>
        </div>
        <div style={{ ...styles.badge, backgroundColor: riskColor }}>
          {risk.final_risk}
        </div>
      </div>

      <div style={styles.summaryBox}>
        <h2 style={styles.summaryTitle}>⚠️ AI 상황 요약</h2>
        <p style={styles.summaryText}>{summary}</p>
      </div>

      <div style={styles.grid}>
        <div style={{ ...styles.card, borderTop: `6px solid ${riskColor}` }}>
          <p style={styles.cardLabel}>종합 위험도</p>
          <h2 style={{ ...styles.cardValue, color: riskColor }}>
            {risk.final_risk}
          </h2>
          <p style={styles.cardSub}>
            가스: {risk.gas_risk} / 온도: {risk.temperature_risk}
          </p>
        </div>

        <div style={styles.card}>
          <p style={styles.cardLabel}>검출 가스</p>
          <h2 style={styles.cardValue}>{gas.gas_type}</h2>
          <p style={styles.cardSub}>{gas.concentration_ppm} ppm</p>
        </div>

        <div style={styles.card}>
          <p style={styles.cardLabel}>설비 온도</p>
          <h2 style={styles.cardValue}>{hottestMachine?.temperature}°C</h2>
          <p style={styles.cardSub}>{hottestMachine?.id}</p>
        </div>

        <div style={styles.card}>
          <p style={styles.cardLabel}>위치</p>
          <h2 style={styles.cardValue}>{sensor.location}</h2>
          <p style={styles.cardSub}>감시 구역</p>
        </div>
      </div>

      <div style={styles.alertGrid}>
        <div style={styles.alertCard}>
          <h3>가스 경보</h3>
          <p>{gas.alert_message}</p>
        </div>
        <div style={styles.alertCard}>
          <h3>과열 경보</h3>
          <p>{thermal.alert_message}</p>
        </div>
      </div>

      <div style={styles.reportCard}>
        <div style={styles.reportHeader}>
          <h2>LLM 사고 대응 리포트</h2>
          <span style={styles.reportTag}>RAG 기반</span>
        </div>
        <div style={styles.reportText}>
          {report || "리포트 생성 중..."}
        </div>
      </div>
    </div>
  );
}

const styles = {
  page: {
    minHeight: "100vh",
    background: "#f4f6f8",
    padding: "36px",
    fontFamily:
      "Inter, Pretendard, Arial, sans-serif",
    color: "#2c454f",
  },
  center: {
    padding: "40px",
    fontFamily: "Arial",
  },
  header: {
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: "24px",
  },
  label: {
    margin: 0,
    color: "#6b7280",
    fontSize: "14px",
    fontWeight: 700,
    letterSpacing: "0.08em",
  },
  title: {
    margin: "6px 0 0",
    fontSize: "34px",
  },
  badge: {
    color: "white",
    padding: "14px 24px",
    borderRadius: "999px",
    fontSize: "22px",
    fontWeight: 800,
    boxShadow: "0 8px 20px rgba(0,0,0,0.15)",
  },
  summaryBox: {
    background: "#4d4d52",
    color: "white",
    borderRadius: "20px",
    padding: "24px",
    marginBottom: "24px",
    boxShadow: "0 10px 24px rgba(0,0,0,0.12)",
  },
  summaryTitle: {
    margin: "0 0 10px",
    fontSize: "22px",
  },
  summaryText: {
    margin: 0,
    fontSize: "18px",
    lineHeight: 1.6,
  },
  grid: {
    display: "grid",
    gridTemplateColumns: "repeat(4, minmax(0, 1fr))",
    gap: "18px",
    marginBottom: "20px",
  },
  card: {
    background: "white",
    borderRadius: "18px",
    padding: "22px",
    boxShadow: "0 8px 20px rgba(15, 23, 42, 0.08)",
  },
  cardLabel: {
    margin: 0,
    color: "#6b7280",
    fontSize: "14px",
    fontWeight: 700,
  },
  cardValue: {
    margin: "10px 0 6px",
    fontSize: "28px",
  },
  cardSub: {
    margin: 0,
    color: "#4b5563",
    fontSize: "15px",
  },
  alertGrid: {
    display: "grid",
    gridTemplateColumns: "1fr 1fr",
    gap: "18px",
    marginBottom: "20px",
  },
  alertCard: {
    background: "#fff7ed",
    border: "1px solid #fed7aa",
    borderRadius: "18px",
    padding: "20px",
  },
  reportCard: {
    background: "white",
    borderRadius: "20px",
    padding: "26px",
    boxShadow: "0 8px 24px rgba(15, 23, 42, 0.1)",
  },
  reportHeader: {
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: "18px",
  },
  reportTag: {
    background: "#e0f2fe",
    color: "#0369a1",
    padding: "6px 12px",
    borderRadius: "999px",
    fontSize: "13px",
    fontWeight: 700,
  },
  reportText: {
    whiteSpace: "pre-line",
    lineHeight: 1.75,
    fontSize: "15.5px",
    color: "#1f2937",
  },
};

export default App;