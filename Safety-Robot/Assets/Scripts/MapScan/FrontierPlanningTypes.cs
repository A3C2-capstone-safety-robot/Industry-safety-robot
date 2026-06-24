using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Frontier 탐색 아키텍처에서 공유하는 계획 결과 타입.
/// 현재 FrontierExplorer의 내부 구조체/리스트를 외부 클래스로 끌어내기 위한 준비 단계다.
/// </summary>
public struct FrontierCandidate
{
    public Vector2Int Block;
    public Vector2Int GoalCell;
    public float Score;
    public int InfoGain;
}

public sealed class FrontierPlan
{
    public Vector2Int TargetBlock { get; private set; }
    public Vector2Int GoalCell { get; private set; }
    public float Score { get; private set; }
    public List<Vector3> Waypoints { get; private set; }

    public FrontierPlan(Vector2Int targetBlock, Vector2Int goalCell, float score, List<Vector3> waypoints)
    {
        TargetBlock = targetBlock;
        GoalCell = goalCell;
        Score = score;
        Waypoints = waypoints;
    }
}
