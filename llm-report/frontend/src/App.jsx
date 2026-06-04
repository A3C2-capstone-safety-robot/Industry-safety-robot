import { useEffect, useRef, useState } from "react";

function App() {
  const [sensor, setSensor] = useState(null);
  const [risk, setRisk] = useState(null);
  const [report, setReport] = useState("");
  const [reportMeta, setReportMeta] = useState(null);
  const [toast, setToast] = useState(null);
  const [error, setError] = useState("");
  const lastReportId = useRef(0);

  // 3초마다 센서 데이터 + 최신 자동 리포트 폴링.
  // 백엔드가 위험 감지/누출원 특정 시 자동 생성한 리포트의 id가 바뀌면
  // 푸시 알림(토스트)을 띄우고 리포트 영역을 자동 갱신한다.
  useEffect(() => {
    let alive = true;

    async function poll() {
      try {
        const sensorRes = await fetch("http://127.0.0.1:8000/sensor");
        const sensorJson = await sensorRes.json();
        if (!alive) return;
        setSensor(sensorJson.sensor_data);
        setRisk(sensorJson.risk);
        setError("");

        const repRes = await fetch("http://127.0.0.1:8000/report/latest");
        const repJson = await repRes.json();
        if (!alive) return;
        if (repJson.id && repJson.id !== lastReportId.current) {
          lastReportId.current = repJson.id;
          setReport(repJson.report || "");
          setReportMeta(repJson);
          setToast(`🚨 사고 리포트 자동 생성됨 (${repJson.created_at})`);
          setTimeout(() => setToast(null), 8000);
        }
      } catch (err) {
        console.error(err);
        if (alive && !sensor) setError("데이터를 불러오지 못했습니다.");
      }
    }

    poll();
    const timer = setInterval(poll, 3000);
    return () => {
      alive = false;
      clearInterval(timer);
    };
  }, []);

  // 수동 생성 버튼 (자동을 기다리지 않고 즉시 생성)
  async function generateNow() {
    setReport("리포트 생성 중...");
    try {
      const res = await fetch("http://127.0.0.1:8000/report");
      const json = await res.json();
      setReport(json.report);
      setReportMeta({
        created_at: new Date().toLocaleTimeString("ko-KR", { hour12: false }),
        trigger: "manual",
      });
    } catch {
      setReport("리포트 생성 실패 — 백엔드 연결을 확인하세요.");
    }
  }

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

  // 위험도 기반 상황 요약 — 실제 이상이 있는 항목만 문장에 포함
  const alerts = [];
  if (risk.gas_risk && risk.gas_risk !== "Normal") {
    alerts.push(`${gas.gas_type} ${Number(gas.concentration_ppm).toFixed(1)}ppm 감지`);
  }
  if (risk.temperature_risk && risk.temperature_risk !== "Normal") {
    const machineId = risk.hottest_machine_id || hottestMachine?.id || "설비";
    const temp = risk.max_temperature ?? hottestMachine?.temperature;
    alerts.push(`${machineId} 과열 (${Number(temp).toFixed(1)}°C)`);
  }

  // 추적/대피 중 래치로 경보 유지 중인 경우 (순간 측정값은 정상이어도)
  const riskHeld = !!risk.risk_held;
  const modeLabel = { GAS_TRACKING: "누출원 추적", EVACUATING: "대피" }[
    sensor.robot_mode
  ];

  let summary;
  if (alerts.length) {
    summary = `${alerts.join(" 및 ")}. ${sensor.location} 접근 통제 및 즉시 대응이 필요합니다.${
      modeLabel ? ` (로봇 ${modeLabel} 진행 중)` : ""
    }`;
  } else if (riskHeld) {
    summary = `사고 대응 진행 중 — 로봇이 ${modeLabel || "대응"} 중입니다. 상황 종료 시까지 경보가 유지됩니다.`;
  } else {
    summary = `이상 징후 없음 — 정상 감시 중입니다. (현재 위치: ${sensor.location})`;
  }
  const summaryIcon = alerts.length || riskHeld ? "⚠️" : "✅";

  // 누출원 상태 요약 (검출 가스 카드)
  const src = sensor.source_found;
  let sourceStatus;
  if (src) {
    sourceStatus = `누출원: ${src.zone || "구역 미상"} ${src.position_ros_xy || ""}`;
  } else if (sensor.robot_mode === "GAS_TRACKING") {
    sourceStatus = "누출원: 추적 중...";
  } else {
    sourceStatus = "누출원: 미특정";
  }
  
   return (
    <div style={styles.page}>
      {toast && <div style={styles.toast}>{toast}</div>}
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
        <h2 style={styles.summaryTitle}>{summaryIcon} AI 상황 요약</h2>
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
          <p style={styles.cardSub}>
            {Number(gas.concentration_ppm).toFixed(1)} ppm
          </p>
          <p style={{ ...styles.cardSub, marginTop: "6px", fontWeight: 700 }}>
            {sourceStatus}
          </p>
        </div>

        <div style={styles.card}>
          <p style={styles.cardLabel}>설비 온도 (최고)</p>
          <h2 style={styles.cardValue}>
            {Number(
              risk.max_temperature ?? hottestMachine?.temperature ?? 0
            ).toFixed(1)}
            °C
          </h2>
          <p style={styles.cardSub}>
            {risk.hottest_machine_id || hottestMachine?.id || "-"}
          </p>
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
          <div style={{ display: "flex", gap: "12px", alignItems: "center" }}>
            {reportMeta?.created_at && (
              <span style={styles.cardSub}>
                {reportMeta.created_at} 생성
                {reportMeta.trigger === "manual" ? " (수동)" : " (자동)"}
              </span>
            )}
            <button style={styles.reportButton} onClick={generateNow}>
              지금 생성
            </button>
            <span style={styles.reportTag}>RAG 기반</span>
          </div>
        </div>
        <div style={styles.reportText}>
          {report ||
            "아직 생성된 리포트가 없습니다 — 위험 감지 또는 누출원 특정 시 자동 생성됩니다."}
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
  reportButton: {
    background: "#0369a1",
    color: "white",
    border: "none",
    borderRadius: "999px",
    padding: "8px 16px",
    fontSize: "13px",
    fontWeight: 700,
    cursor: "pointer",
  },
  toast: {
    position: "fixed",
    top: "24px",
    right: "24px",
    zIndex: 1000,
    background: "#dc2626",
    color: "white",
    padding: "16px 22px",
    borderRadius: "14px",
    fontSize: "16px",
    fontWeight: 700,
    boxShadow: "0 12px 30px rgba(220, 38, 38, 0.45)",
  },
};

export default App;