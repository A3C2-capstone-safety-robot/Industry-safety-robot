// GasLeakManager.cs
// 가스 누출 시뮬레이션 관리자 — 캔버스 버튼 API + 랜덤 자동 누출
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class GasLeakManager : MonoBehaviour
{
    [Header("랜덤 누출 설정")]
    public bool enableRandomLeak = false;          // 랜덤 누출 모드 (UI 토글 버튼으로 제어)
    public float randomLeakMinInterval = 30f;      // 최소 대기 시간 (초)
    public float randomLeakMaxInterval = 120f;     // 최대 대기 시간 (초)
    public float randomLeakDuration = 60f;         // 랜덤 누출 지속 시간 (초)

    [Header("선택: 랜덤 누출 토글 버튼 색상")]
    [Tooltip("버튼의 Image 를 연결하면 ON/OFF 에 따라 색이 바뀜")]
    public Image randomButtonImage;
    public Color randomOnColor = new Color(0.30f, 0.78f, 0.40f);   // ON: 초록
    public Color randomOffColor = new Color(0.85f, 0.85f, 0.85f);  // OFF: 회색
    [Tooltip("버튼 글자(선택). 연결하면 ON/OFF 텍스트로 바뀜")]
    public Text randomButtonLabel;

    // 씬의 모든 누출원 자동 탐지
    private GaussianPlumeModel[] allLeakPoints;
    private float nextRandomLeakTime;
    private int currentRandomIndex = -1;
    private float randomLeakEndTime;

    void Start()
    {
        // 씬에서 모든 GaussianPlumeModel 자동으로 찾기
        RefreshLeakPoints();
        ScheduleNextRandomLeak();
        UpdateRandomButtonVisual();
    }

    // ── 랜덤 누출 모드 토글 — UI 버튼의 OnClick 에 연결 ──
    public void ToggleRandomLeak()
    {
        SetRandomLeakEnabled(!enableRandomLeak);
    }

    public void SetRandomLeakEnabled(bool on)
    {
        enableRandomLeak = on;

        if (on)
        {
            // 켜는 즉시 터지지 않게 새로 스케줄
            ScheduleNextRandomLeak();
        }
        else if (currentRandomIndex >= 0)
        {
            // 진행 중이던 랜덤 누출도 정리
            StopLeak(currentRandomIndex);
            currentRandomIndex = -1;
        }

        UpdateRandomButtonVisual();
        Debug.Log($"[GasLeakManager] 랜덤 누출 모드: {(on ? "ON" : "OFF")}");
    }

    void UpdateRandomButtonVisual()
    {
        if (randomButtonImage != null)
            randomButtonImage.color = enableRandomLeak ? randomOnColor : randomOffColor;
        if (randomButtonLabel != null)
            randomButtonLabel.text = enableRandomLeak ? "랜덤 누출 ON" : "랜덤 누출 OFF";
    }

    // 누출원 목록 갱신
    public void RefreshLeakPoints()
    {
        allLeakPoints = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);
        Debug.Log($"[GasLeakManager] {allLeakPoints.Length}개의 누출 지점 발견");
    }

    void Update()
    {
        // 랜덤 누출 처리 (자동 스케줄)
        if (enableRandomLeak)
        {
            HandleRandomLeak();
        }
    }

    // ============================================================
    //  랜덤 누출
    // ============================================================

    void HandleRandomLeak()
    {
        if (allLeakPoints == null || allLeakPoints.Length == 0) return;

        // 랜덤 누출 시작 시간 도달
        if (Time.time >= nextRandomLeakTime && currentRandomIndex == -1)
        {
            // 현재 누출 중이 아닌 머신 중에서 랜덤 선택
            List<int> available = new List<int>();
            for (int i = 0; i < allLeakPoints.Length; i++)
            {
                if (!allLeakPoints[i].isLeaking)
                    available.Add(i);
            }

            if (available.Count > 0)
            {
                currentRandomIndex = available[Random.Range(0, available.Count)];
                StartLeak(currentRandomIndex);
                randomLeakEndTime = Time.time + randomLeakDuration;
                Debug.Log($"[GasLeakManager] 랜덤 누출 시작: {allLeakPoints[currentRandomIndex].gameObject.name}");
            }
        }

        // 랜덤 누출 종료
        if (currentRandomIndex >= 0 && Time.time >= randomLeakEndTime)
        {
            StopLeak(currentRandomIndex);
            Debug.Log($"[GasLeakManager] 랜덤 누출 종료: {allLeakPoints[currentRandomIndex].gameObject.name}");
            currentRandomIndex = -1;
            ScheduleNextRandomLeak();
        }
    }

    void ScheduleNextRandomLeak()
    {
        nextRandomLeakTime = Time.time + Random.Range(randomLeakMinInterval, randomLeakMaxInterval);
    }

    // ============================================================
    //  누출 제어 (UI, 외부 호출용)
    // ============================================================

    public void StartLeak(int index)
    {
        if (index < 0 || index >= allLeakPoints.Length) return;
        allLeakPoints[index].StartLeak();
    }

    public void StopLeak(int index)
    {
        if (index < 0 || index >= allLeakPoints.Length) return;
        allLeakPoints[index].StopLeak();
    }

    public void StopAllLeaks()
    {
        for (int i = 0; i < allLeakPoints.Length; i++)
        {
            allLeakPoints[i].StopLeak();
        }
        currentRandomIndex = -1;
    }

    // ============================================================
    //  캔버스 버튼용 API (Button OnClick에 연결)
    // ============================================================

    // 누출 중이 아닌 기계 하나를 랜덤으로 골라 누출 시작
    public void StartRandomLeakOnce()
    {
        if (allLeakPoints == null || allLeakPoints.Length == 0)
            RefreshLeakPoints();

        List<int> available = new List<int>();
        for (int i = 0; i < allLeakPoints.Length; i++)
            if (!allLeakPoints[i].isLeaking)
                available.Add(i);

        if (available.Count == 0)
        {
            Debug.Log("[GasLeakManager] 모든 기계가 이미 누출 중 — 랜덤 누출 불가");
            return;
        }

        int idx = available[Random.Range(0, available.Count)];
        StartLeak(idx);
        Debug.Log($"[GasLeakManager] (버튼) 랜덤 누출 시작: {allLeakPoints[idx].gameObject.name}");
    }
}
