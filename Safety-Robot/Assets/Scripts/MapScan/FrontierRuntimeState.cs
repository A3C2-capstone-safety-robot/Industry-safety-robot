using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Frontier 탐색기의 런타임 상태.
/// 블랙리스트, 방문 frontier, 현재 목표처럼 planner와 주행기가 같이 보는 상태를 한곳에 모은다.
/// </summary>
public sealed class FrontierRuntimeState
{
    public readonly List<Vector2Int> Blacklist = new List<Vector2Int>();
    public readonly HashSet<Vector2Int> VisitedFrontiers = new HashSet<Vector2Int>();

    public Vector2Int CurrentTargetBlock { get; set; }
    public int RelaxCount { get; set; }

    public void ResetForNewRun(bool keepVisitedFrontiers)
    {
        Blacklist.Clear();
        RelaxCount = 0;

        if (!keepVisitedFrontiers)
            VisitedFrontiers.Clear();
    }

    public void BlacklistNeighborhood(Vector2Int center, int radius)
    {
        for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
        {
            var block = new Vector2Int(center.x + dx, center.y + dy);
            if (!Blacklist.Contains(block))
                Blacklist.Add(block);
        }
    }
}
