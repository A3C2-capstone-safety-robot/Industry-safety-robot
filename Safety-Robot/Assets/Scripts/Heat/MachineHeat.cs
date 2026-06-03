// MachineHeat.cs
using UnityEngine;

public class MachineHeat : MonoBehaviour
{
    [Header("기본 온도 설정")]
    public string machineId = "machine_1";
    public float baselineTemp = 65f;
    public float currentTemp;
    public float fluctuation = 1.0f;

    [Header("과열 설정")]
    public float cautionThreshold = 80f;
    public float warningThreshold = 100f;
    public float dangerThreshold  = 120f;
    public float maxTemp          = 180f;

    [Header("센서 노이즈 설정")]
    public float noiseAmplitude = 0.5f; // ±0.5°C 측정 오차

    [Header("시각화")]
    public Renderer machineRenderer;
    public Color normalColor  = new Color(0.2f, 0.4f, 0.8f);
    public Color cautionColor = new Color(1f, 1f, 0f);
    public Color warningColor = new Color(1f, 0.5f, 0f);
    public Color dangerColor  = new Color(1f, 0f, 0f);

    private bool  isOverheating = false;
    private float overheatRate  = 0f;
    private MaterialPropertyBlock propBlock;

    public enum HeatStatus { Normal, Caution, Warning, Danger }
    public HeatStatus Status { get; private set; }

    void Start()
    {
        currentTemp = baselineTemp;
        propBlock   = new MaterialPropertyBlock();
        if (machineRenderer == null)
            machineRenderer = GetComponent<Renderer>();
        // 루트에 Renderer 가 없으면 자식 메쉬에서 찾음 (새 기계 모델 대응)
        if (machineRenderer == null)
            machineRenderer = GetComponentInChildren<Renderer>();
        if (machineRenderer == null)
            Debug.LogWarning($"[MachineHeat] {machineId}: Renderer 를 못 찾음 — 색 변화 비활성. " +
                             "Inspector 의 Machine Renderer 칸에 메쉬 오브젝트를 연결하세요.", this);
    }

    void Update()
    {
        if (isOverheating)
        {
            currentTemp += overheatRate * Time.deltaTime;
            currentTemp  = Mathf.Min(currentTemp, maxTemp);
        }
        else
        {
            currentTemp += Random.Range(-fluctuation, fluctuation) * Time.deltaTime;
            currentTemp  = Mathf.Lerp(currentTemp, baselineTemp, 0.01f);
        }

        if      (currentTemp >= dangerThreshold)  Status = HeatStatus.Danger;
        else if (currentTemp >= warningThreshold) Status = HeatStatus.Warning;
        else if (currentTemp >= cautionThreshold) Status = HeatStatus.Caution;
        else                                      Status = HeatStatus.Normal;

        UpdateVisual();
    }

    void UpdateVisual()
    {
        // Renderer 가 없으면 색 변화 스킵 (크래시 방지)
        if (machineRenderer == null) return;
        if (propBlock == null) propBlock = new MaterialPropertyBlock();

        Color targetColor;
        float t;

        if (currentTemp <= cautionThreshold)
        {
            t = Mathf.InverseLerp(baselineTemp, cautionThreshold, currentTemp);
            targetColor = Color.Lerp(normalColor, cautionColor, t);
        }
        else if (currentTemp <= warningThreshold)
        {
            t = Mathf.InverseLerp(cautionThreshold, warningThreshold, currentTemp);
            targetColor = Color.Lerp(cautionColor, warningColor, t);
        }
        else
        {
            t = Mathf.InverseLerp(warningThreshold, dangerThreshold, currentTemp);
            targetColor = Color.Lerp(warningColor, dangerColor, t);
        }

        machineRenderer.GetPropertyBlock(propBlock);
        propBlock.SetColor("_EmissionColor", targetColor * 2f);
        propBlock.SetColor("_BaseColor", targetColor); // URP로 수정하면 변경해야할 부분
        machineRenderer.SetPropertyBlock(propBlock);
    }

    /// <summary>센서 노이즈(±noiseAmplitude)가 적용된 온도값 반환.</summary>
    public float GetNoisyTemp()
    {
        return currentTemp + Random.Range(-noiseAmplitude, noiseAmplitude);
    }

    public void StartOverheat(float rate = 2f)
    {
        isOverheating = true;
        overheatRate  = rate;
    }

    public void StopOverheat()  { isOverheating = false; }
    public void SetTemperature(float temp) { currentTemp = temp; }
}
