#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "PathNode.h"
#include <vector>

constexpr int GRID_WIDTH = 40;
constexpr int GRID_HEIGHT = 40;
constexpr float SQRT_2 = 1.41421356237f;
constexpr float NODE_DIAGONAL_COST = SQRT_2;
constexpr float NODE_STRAIGHT_COST = 1.0f;
constexpr float INF = 1000000.0f;

constexpr float cfmin(const float a, const float b) {
    return (a < b) ? a : b;
}

constexpr float cfmax(const float a, const float b) {
    return (a > b) ? a : b;
}

constexpr float HeuristicOctile(const float diffX, const float diffY)
{
	return cfmin(diffX, diffY) * SQRT_2 + cfmax(diffX, diffY) - cfmin(diffX, diffY);
}

inline float HeuristicManhattan(const GridPos& inStart, const GridPos& inEnd)
{
    const float diffX = std::fabsf(static_cast<float>(inStart.row - inEnd.row));
    const float diffY = std::fabsf(static_cast<float>(inStart.col - inEnd.col));
    return diffX + diffY;
}


inline float HeuristicChebyshev(const GridPos& inStart, const GridPos& inEnd)
{
    const float diffX = std::fabsf(static_cast<float>(inStart.row - inEnd.row));
    const float diffY = std::fabsf(static_cast<float>(inStart.col - inEnd.col));
    return std::fmax(diffX, diffY);
}

inline float HeuristicEuclidean(const GridPos& inStart, const GridPos& inEnd)
{
    const float diffX = std::fabsf(static_cast<float>(inStart.row - inEnd.row));
    const float diffY = std::fabsf(static_cast<float>(inStart.col - inEnd.col));
    return std::sqrtf(std::powf(diffX, 2) + std::powf(diffY, 2));
}

inline float HeuristicInconsistent(const GridPos& inStart, const GridPos& inEnd)
{
    const float diffX = std::fabsf(static_cast<float>(inStart.row - inEnd.row));
    const float diffY = std::fabsf(static_cast<float>(inStart.col - inEnd.col));
    return ((inStart.row + inStart.col % 2) > 0) ? std::sqrtf(std::powf(diffX, 2) + std::powf(diffY, 2)) : 0.0f;
}

inline float CalculateHeuristic(const GridPos& inStart, Heuristic _heuristic, const GridPos& inEnd)
{
    const float diffX = std::fabsf(static_cast<float>(inStart.row - inEnd.row));
    const float diffY = std::fabsf(static_cast<float>(inStart.col - inEnd.col));

    switch (_heuristic)
    {
    case Heuristic::MANHATTAN:
    {
        return HeuristicManhattan(inStart, inEnd);
    }

    case Heuristic::EUCLIDEAN:
    {
        return HeuristicEuclidean(inStart, inEnd);
    }

    case Heuristic::CHEBYSHEV:
    {
        return HeuristicChebyshev(inStart, inEnd);
    }

    case Heuristic::OCTILE:
    {
        return HeuristicOctile(diffX, diffY);
    }

    case Heuristic::INCONSISTENT:
    {
        return HeuristicInconsistent(inStart, inEnd);
    }

    default:
        return 0.0f;
    }
}

constexpr std::array<GridPos, 8> neighbourOffsets
{
    GridPos{ -1, -1 },
    GridPos{ -1, 0 },
    GridPos{ -1, 1 },
    GridPos{ 0, -1 },
    GridPos{ 0, 1 },
    GridPos{ 1, -1 },
    GridPos{ 1, 0 },
    GridPos{ 1, 1 }
};

constexpr std::array<float, 8> neighbourCost
{
    NODE_DIAGONAL_COST,
    NODE_STRAIGHT_COST,
    NODE_DIAGONAL_COST,
    NODE_STRAIGHT_COST,
    NODE_STRAIGHT_COST,
    NODE_DIAGONAL_COST,
    NODE_STRAIGHT_COST,
    NODE_DIAGONAL_COST
};

using Grid = std::array<std::array<PathNode, GRID_WIDTH>, GRID_HEIGHT>;

struct PathNodeCompare
{
	bool operator()(const PathNode* lhs, const PathNode* rhs) const
	{
		return lhs->finalCost > rhs->finalCost;
	}
};

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */

private:


    Grid _grid {};
	std::vector<PathNode*> _openList; //TODO: Change to tree // TODO: Put it on the stack
    PathNode* _goalNode{ nullptr };
    PathNode* _parentNode{ nullptr };
    Heuristic _heuristic{ Heuristic::MANHATTAN };
    float _weight = 1.0f;
    bool _debugColoring{ false };
    bool _singleStep{ false };

	// Pathfinding Functions
    void UpdateAllNodeNeighbours();
	void UpdateNodeAccessibleNeighbours(PathNode* inPathNode);
	void AddAllNeighboursToOpenList(PathNode* inPathNode);

    //Create the Path Node Compare Function
    //float CalculateHeuristic(const GridPos& inStart);
    void ResetGrid();

	void RubberBandPath(WaypointList& inPath);
    bool IsNodeDeletable(const GridPos& inCurrent ,const GridPos& inStart, const GridPos& inEnd);

	void SmoothPath(WaypointList& inPath);

	float GridPosDistance(const GridPos& inStart, const GridPos& inEnd);

    void AddBackNodes(WaypointList& inPath);
};