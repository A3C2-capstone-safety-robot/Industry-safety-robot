using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Frontier 후보 탐색과 글로벌 경로계획을 담당하는 순수 로직 클래스.
/// Unity 물리나 ROS 구독은 FrontierExplorer에 남기고, 맵 해석과 목표 선택만 분리한다.
/// </summary>
public sealed class FrontierPlanner
{
    static readonly Vector2Int[] Dirs4 =
    {
        new Vector2Int(1, 0), new Vector2Int(-1, 0),
        new Vector2Int(0, 1), new Vector2Int(0, -1)
    };

    public List<FrontierCandidate> FindCandidates(
        ExplorationMapSnapshot map,
        Vector3 robotWorldPos,
        float worldY,
        FrontierPlanningSettings settings,
        FrontierRuntimeState state)
    {
        settings.Clamp();

        int bs = settings.blockSize;
        int bW = map.BlockWidth(bs);
        int bH = map.BlockHeight(bs);
        Vector2Int startBlock = map.CellToBlock(map.WorldToCell(robotWorldPos), bs);

        var result = new List<FrontierCandidate>();
        if (startBlock.x < 0 || startBlock.y < 0 || startBlock.x >= bW || startBlock.y >= bH)
            return result;

        var visited = new HashSet<Vector2Int>();
        var queue = new Queue<Vector2Int>();
        visited.Add(startBlock);
        queue.Enqueue(startBlock);

        while (queue.Count > 0)
        {
            Vector2Int current = queue.Dequeue();
            Vector2Int center = map.BlockToCenter(current, bs);
            int cellValue = map.Cell(center.x, center.y);

            if (cellValue == 0
                && HasUnknownNeighborBlock(map, current, bs)
                && !state.Blacklist.Contains(current)
                && IsSafe(map, center, settings.safetyRadius))
            {
                int infoGain = CountUnknown(map, center, bs);
                if (infoGain >= settings.minInfoGain)
                {
                    Vector2Int goalCell;
                    int openness;
                    if (TryFindGoalCellInBlock(map, current, bs, robotWorldPos, settings, out goalCell, out openness))
                    {
                        float dist = Vector3.Distance(robotWorldPos, map.CellToWorld(goalCell.x, goalCell.y, worldY));
                        float score = -settings.distanceWeight * dist
                            + settings.infoGainWeight * infoGain
                            + settings.goalOpennessWeight * openness;

                        int visitedDist = MinDistToVisited(current, state);
                        if (visitedDist <= settings.visitedPenaltyRadius)
                            score -= settings.visitedPenaltyWeight * (settings.visitedPenaltyRadius - visitedDist + 1);

                        result.Add(new FrontierCandidate
                        {
                            Block = current,
                            GoalCell = goalCell,
                            Score = score,
                            InfoGain = infoGain
                        });
                    }
                }
            }

            foreach (var dir in Dirs4)
            {
                Vector2Int next = current + dir;
                if (visited.Contains(next)) continue;
                if (next.x < 0 || next.y < 0 || next.x >= bW || next.y >= bH) continue;

                Vector2Int nextCenter = map.BlockToCenter(next, bs);
                if (map.Cell(nextCenter.x, nextCenter.y) != 0) continue;

                visited.Add(next);
                queue.Enqueue(next);
            }
        }

        return result;
    }

    public FrontierPlan TryBuildPlan(
        ExplorationMapSnapshot map,
        Vector3 robotWorldPos,
        float worldY,
        FrontierPlanningSettings settings,
        FrontierRuntimeState state,
        FrontierCandidate candidate,
        Func<Vector3, Vector3, bool> segmentValidator)
    {
        settings.Clamp();
        state.CurrentTargetBlock = candidate.Block;

        List<Vector3> waypoints = BuildPath(map, robotWorldPos, worldY, candidate.GoalCell, settings, segmentValidator);
        if (waypoints == null || waypoints.Count == 0)
        {
            state.BlacklistNeighborhood(candidate.Block, settings.blacklistNeighborRadius);
            return null;
        }

        return new FrontierPlan(candidate.Block, candidate.GoalCell, candidate.Score, waypoints);
    }

    bool HasUnknownNeighborBlock(ExplorationMapSnapshot map, Vector2Int block, int blockSize)
    {
        foreach (var dir in Dirs4)
        {
            Vector2Int nc = map.BlockToCenter(block + dir, blockSize);
            if (map.Cell(nc.x, nc.y) < 0) return true;
        }
        return false;
    }

    bool IsSafe(ExplorationMapSnapshot map, Vector2Int cell, int safetyRadius)
    {
        for (int dy = -safetyRadius; dy <= safetyRadius; dy++)
        for (int dx = -safetyRadius; dx <= safetyRadius; dx++)
        {
            if (dx * dx + dy * dy > safetyRadius * safetyRadius) continue;
            if (map.Cell(cell.x + dx, cell.y + dy) > 50) return false;
        }
        return true;
    }

    bool IsClear(ExplorationMapSnapshot map, int cx, int cy, int pathInflation)
    {
        for (int dy = -pathInflation; dy <= pathInflation; dy++)
        for (int dx = -pathInflation; dx <= pathInflation; dx++)
        {
            if (map.Cell(cx + dx, cy + dy) > 50) return false;
        }
        return true;
    }

    int CountUnknown(ExplorationMapSnapshot map, Vector2Int cell, int radius)
    {
        int count = 0;
        for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
        {
            if (map.Cell(cell.x + dx, cell.y + dy) < 0)
                count++;
        }
        return count;
    }

    int CountOpenNeighbors(ExplorationMapSnapshot map, Vector2Int cell, int radius)
    {
        int count = 0;
        for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
        {
            if (dx == 0 && dy == 0) continue;
            if (map.Cell(cell.x + dx, cell.y + dy) == 0)
                count++;
        }
        return count;
    }

    int MinDistToVisited(Vector2Int block, FrontierRuntimeState state)
    {
        int best = int.MaxValue;
        foreach (var visited in state.VisitedFrontiers)
        {
            int dist = Mathf.Abs(block.x - visited.x) + Mathf.Abs(block.y - visited.y);
            if (dist < best) best = dist;
        }
        return best;
    }

    bool TryFindGoalCellInBlock(
        ExplorationMapSnapshot map,
        Vector2Int block,
        int blockSize,
        Vector3 robotWorldPos,
        FrontierPlanningSettings settings,
        out Vector2Int goalCell,
        out int goalOpenness)
    {
        int minX = Mathf.Max(0, block.x * blockSize);
        int minY = Mathf.Max(0, block.y * blockSize);
        int maxX = Mathf.Min(map.Width - 1, (block.x + 1) * blockSize - 1);
        int maxY = Mathf.Min(map.Height - 1, (block.y + 1) * blockSize - 1);

        Vector2Int robotCell = map.WorldToCell(robotWorldPos);
        bool found = false;
        int bestUnknown = -1;
        int bestOpenness = -1;
        float bestDistSq = float.MaxValue;
        goalCell = Vector2Int.zero;
        goalOpenness = -1;

        for (int y = minY; y <= maxY; y++)
        for (int x = minX; x <= maxX; x++)
        {
            if (map.Cell(x, y) != 0) continue;

            Vector2Int cell = new Vector2Int(x, y);
            if (!IsSafe(map, cell, settings.safetyRadius)) continue;

            int unknown = CountUnknown(map, cell, 1);
            int openness = CountOpenNeighbors(map, cell, settings.goalClearanceRadius);
            if (openness < settings.minGoalOpenNeighbors) continue;
            float distSq = (cell - robotCell).sqrMagnitude;

            if (!found
                || unknown > bestUnknown
                || (unknown == bestUnknown && openness > bestOpenness)
                || (unknown == bestUnknown && openness == bestOpenness && distSq < bestDistSq))
            {
                found = true;
                bestUnknown = unknown;
                bestOpenness = openness;
                bestDistSq = distSq;
                goalCell = cell;
                goalOpenness = openness;
            }
        }

        return found;
    }

    List<Vector2Int> BuildBlockPath(
        ExplorationMapSnapshot map,
        Vector2Int start,
        Vector2Int goal,
        int blockSize,
        int pathInflation,
        bool inflated)
    {
        int bW = map.BlockWidth(blockSize);
        int bH = map.BlockHeight(blockSize);
        var visited = new HashSet<Vector2Int>();
        var queue = new Queue<Vector2Int>();
        var parent = new Dictionary<Vector2Int, Vector2Int>();

        visited.Add(start);
        queue.Enqueue(start);

        while (queue.Count > 0)
        {
            Vector2Int current = queue.Dequeue();
            if (current == goal)
            {
                var path = new List<Vector2Int>();
                for (var n = current; n != start; n = parent[n]) path.Add(n);
                path.Add(start);
                path.Reverse();
                return path;
            }

            foreach (var dir in Dirs4)
            {
                Vector2Int next = current + dir;
                if (visited.Contains(next)) continue;
                if (next.x < 0 || next.y < 0 || next.x >= bW || next.y >= bH) continue;

                Vector2Int nextCenter = map.BlockToCenter(next, blockSize);
                int value = map.Cell(nextCenter.x, nextCenter.y);
                bool ok = inflated ? (value == 0 && IsClear(map, nextCenter.x, nextCenter.y, pathInflation)) : (value == 0);
                if (!ok) continue;

                visited.Add(next);
                parent[next] = current;
                queue.Enqueue(next);
            }
        }

        return null;
    }

    List<Vector3> BuildPath(
        ExplorationMapSnapshot map,
        Vector3 robotWorldPos,
        float worldY,
        Vector2Int goalCell,
        FrontierPlanningSettings settings,
        Func<Vector3, Vector3, bool> segmentValidator)
    {
        int blockSize = settings.pathBlockSize;
        Vector2Int startBlock = map.CellToBlock(map.WorldToCell(robotWorldPos), blockSize);
        Vector2Int goalBlock = map.CellToBlock(goalCell, blockSize);

        List<Vector2Int> blocks =
            BuildBlockPath(map, startBlock, goalBlock, blockSize, settings.pathInflation, true)
            ?? BuildBlockPath(map, startBlock, goalBlock, blockSize, settings.pathInflation, false);

        if (blocks == null) return null;

        var points = new List<Vector3>();
        foreach (var block in blocks)
        {
            Vector2Int center = map.BlockToCenter(block, blockSize);
            points.Add(map.CellToWorld(center.x, center.y, worldY));
        }

        if (segmentValidator != null)
        {
            for (int i = 0; i < points.Count - 1; i++)
            {
                if (!segmentValidator(points[i], points[i + 1]))
                    return null;
            }
        }

        if (points.Count > settings.pathWaypoints)
        {
            var sampled = new List<Vector3>();
            float step = (float)(points.Count - 1) / (settings.pathWaypoints - 1);
            for (int i = 0; i < settings.pathWaypoints; i++)
                sampled.Add(points[Mathf.RoundToInt(i * step)]);
            points = sampled;
        }

        return points;
    }
}
