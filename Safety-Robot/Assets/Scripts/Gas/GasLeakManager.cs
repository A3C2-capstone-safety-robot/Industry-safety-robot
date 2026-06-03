// GasLeakManager.cs
// 가스 누출 시뮬레이션 관리자 — UI 버튼 + 랜덤 자동 누출
using UnityEngine;
using System.Collections.Generic;

public class GasLeakManager : MonoBehaviour
{
    [Header("랜덤 누출 설정")]
    public bool enableRandomLeak = true;           // 랜덤 누출 활성화
    public float randomLeakMinInterval = 30f;      // 최소 대기 시간 (초)
    public float randomLeakMaxInterval = 120f;     // 최대 대기 시간 (초)
    public float randomLeakDuration = 60f;         // 랜덤 누출 지속 시간 (초)

    [Header("UI 설정")]
    public bool showUI = true;
    public KeyCode toggleUIKey = KeyCode.G;        // UI 표시/숨김 토글 키 (가스 = G)

    // 씬의 모든 누출원 자동 탐지
    private GaussianPlumeModel[] allLeakPoints;
    private float nextRandomLeakTime;
    private int currentRandomIndex = -1;
    private float randomLeakEndTime;

    // UI 관련
    private bool uiVisible = true;
    private Vector2 scrollPosition;

    void Start()
    {
        // 씬에서 모든 GaussianPlumeModel 자동으로 찾기
        RefreshLeakPoints();
        ScheduleNextRandomLeak();
    }

    // 누출원 목록 갱신
    public void RefreshLeakPoints()
    {
        allLeakPoints = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);
        Debug.Log($"[GasLeakManager] {allLeakPoints.Length}개의 누출 지점 발견");
    }

    void Update()
    {
        // UI 토글
        if (Input.GetKeyDown(toggleUIKey))
        {
            uiVisible = !uiVisible;
        }

        // 랜덤 누출 처리
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
    //  UI (Play 모드에서 화면에 버튼 표시)
    // ============================================================

    void OnGUI()
    {
        if (!showUI || !uiVisible || allLeakPoints == null) return;

        float panelWidth = 280f;
        float panelX = Screen.width - panelWidth - 20f;
        float panelY = 20f;

        // 배경 패널
        GUI.Box(new Rect(panelX, panelY, panelWidth, GetPanelHeight()), "");

        GUILayout.BeginArea(new Rect(panelX + 10, panelY + 10, panelWidth - 20, GetPanelHeight() - 20));

        // 제목
        GUIStyle titleStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 16,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter
        };
        GUILayout.Label("Gas Leak Control", titleStyle);
        GUILayout.Space(10);

        // 전체 중지 버튼
        GUI.backgroundColor = new Color(1f, 0.4f, 0.4f);
        if (GUILayout.Button("Stop All Leaks", GUILayout.Height(30)))
        {
            StopAllLeaks();
        }
        GUI.backgroundColor = Color.white;
        GUILayout.Space(5);

        // 랜덤 누출 토글
        enableRandomLeak = GUILayout.Toggle(enableRandomLeak, " Enable Random Leak");
        GUILayout.Space(10);

        // 각 머신별 버튼
        scrollPosition = GUILayout.BeginScrollView(scrollPosition, GUILayout.Height(300));

        for (int i = 0; i < allLeakPoints.Length; i++)
        {
            var plume = allLeakPoints[i];
            string machineName = plume.gameObject.name;
            bool isLeaking = plume.isLeaking;

            GUILayout.BeginHorizontal();

            // 상태 표시
            GUIStyle statusStyle = new GUIStyle(GUI.skin.label)
            {
                fontStyle = FontStyle.Bold,
                normal = { textColor = isLeaking ? Color.red : Color.green }
            };
            GUILayout.Label(isLeaking ? "●" : "○", statusStyle, GUILayout.Width(20));

            // 머신 이름 + 가스 타입
            GUILayout.Label($"{machineName} ({plume.gasType})", GUILayout.Width(140));

            // 시작/중지 버튼
            if (isLeaking)
            {
                GUI.backgroundColor = new Color(1f, 0.6f, 0.6f);
                if (GUILayout.Button("Stop", GUILayout.Width(60)))
                {
                    StopLeak(i);
                }
            }
            else
            {
                GUI.backgroundColor = new Color(0.6f, 1f, 0.6f);
                if (GUILayout.Button("Start", GUILayout.Width(60)))
                {
                    StartLeak(i);
                }
            }
            GUI.backgroundColor = Color.white;

            GUILayout.EndHorizontal();
            GUILayout.Space(2);
        }

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    float GetPanelHeight()
    {
        int count = allLeakPoints != null ? allLeakPoints.Length : 0;
        return Mathf.Min(100 + count * 28, 450);
    }
}
