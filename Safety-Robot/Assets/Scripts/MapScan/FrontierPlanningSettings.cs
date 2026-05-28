using System;
using UnityEngine;

/// <summary>
/// Frontier 탐색기에서 글로벌 목표 선택과 경로계획에 쓰는 파라미터 묶음.
/// MonoBehaviour가 아닌 별도 설정 객체로 분리해 두면 planner 유닛 테스트와 재사용이 쉬워진다.
/// </summary>
[Serializable]
public class FrontierPlanningSettings
{
    [Header("Frontier Search")]
    public int blockSize = 10;
    public float distanceWeight = 1f;
    public float infoGainWeight = 2f;
    public int safetyRadius = 3;
    public int minInfoGain = 5;
    public float visitedPenaltyWeight = 3f;
    public int visitedPenaltyRadius = 2;

    [Header("Path Search")]
    public int pathBlockSize = 1;
    public int pathInflation = 0;
    public int pathWaypoints = 12;
    public int maxPlanAttemptsPerSearch = 8;
    public int blacklistNeighborRadius = 1;
    public int goalClearanceRadius = 1;
    public int minGoalOpenNeighbors = 5;
    public float goalOpennessWeight = 1.5f;

    public void Clamp()
    {
        blockSize = Mathf.Max(1, blockSize);
        pathBlockSize = Mathf.Max(1, pathBlockSize);
        pathWaypoints = Mathf.Max(2, pathWaypoints);
        safetyRadius = Mathf.Max(0, safetyRadius);
        pathInflation = Mathf.Max(0, pathInflation);
        minInfoGain = Mathf.Max(0, minInfoGain);
        visitedPenaltyRadius = Mathf.Max(0, visitedPenaltyRadius);
        maxPlanAttemptsPerSearch = Mathf.Max(1, maxPlanAttemptsPerSearch);
        blacklistNeighborRadius = Mathf.Max(0, blacklistNeighborRadius);
        goalClearanceRadius = Mathf.Max(0, goalClearanceRadius);
        minGoalOpenNeighbors = Mathf.Max(0, minGoalOpenNeighbors);
    }
}
