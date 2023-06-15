#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "PathNode.h"
#include "limits.h"
#include <vector>

constexpr int GRID_WIDTH = 40;
constexpr int GRID_HEIGHT = 40;
constexpr int SQRT_2 = 14124;
constexpr int NODE_DIAGONAL_COST = SQRT_2;
constexpr int NODE_STRAIGHT_COST = 10000;
constexpr int OCTILE_MIN = NODE_DIAGONAL_COST - NODE_STRAIGHT_COST;
constexpr int INF = 100000000;

constexpr int cfmin(const int a, const int b) {
    return (a < b) ? a : b;
}

constexpr int cfmax(const int a, const int b) {
    return (a > b) ? a : b;
}

constexpr int HeuristicOctile(const int diffX, const int diffY)
{
	return OCTILE_MIN * cfmin(diffX, diffY) + NODE_STRAIGHT_COST * cfmax(diffX, diffY);
}

inline int HeuristicManhattan(const GridPos& inStart, const GridPos& inEnd)
{
    const int diffX = std::abs(inStart.row - inEnd.row);
    const int diffY = std::abs(inStart.col - inEnd.col);
    return diffX + diffY;
}

inline int HeuristicChebyshev(const GridPos& inStart, const GridPos& inEnd)
{
    const int diffX = std::abs(inStart.row - inEnd.row);
    const int diffY = std::abs(inStart.col - inEnd.col);
    return std::max(diffX, diffY);
}

inline int HeuristicEuclidean(const GridPos& inStart, const GridPos& inEnd)
{
    const int diffX = std::abs(inStart.row - inEnd.row);
    const int diffY = std::abs(inStart.col - inEnd.col);
	return static_cast<int>(sqrtf(static_cast<float>(diffX * diffX + diffY * diffY)));
}

inline int HeuristicInconsistent(const GridPos& inStart, const GridPos& inEnd) //TODO : FIX THIS ASAP 
{
    return ((inStart.row + inStart.col) % 2 > 0) ? HeuristicEuclidean(inStart,inEnd) : 0;
}

inline int CalculateHeuristic(const GridPos& inStart, Heuristic _heuristic, const GridPos& inEnd)
{
    const int diffX = std::abs(inStart.row - inEnd.row);
    const int diffY = std::abs(inStart.col - inEnd.col);

    if (_heuristic == Heuristic::OCTILE)
        return HeuristicOctile(diffX, diffY);

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


    case Heuristic::INCONSISTENT:
    {
        return HeuristicInconsistent(inStart, inEnd);
    }

    default:
        return 0;
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

constexpr std::array<int, 8> neighbourCost
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

constexpr int V = 1600;

using Grid = std::array<std::array<PathNode, GRID_WIDTH>, GRID_HEIGHT>;

struct PathNodeCompare
{
	bool operator()(const PathNode* lhs, const PathNode* rhs) const
	{
		return lhs->finalCost > rhs->finalCost;
	}
};

class UnsortedList
{
public:
    std::array<PathNode*, 158> _list{};
    int gridSize = 0;

    void Push(PathNode* inPathNode);
    PathNode* FindCheapestNodeAndPop();
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

    //Floyd Warshall Variables
    int distance[V][V]{};
    int previous[V][V]{}; //tracking the previous Index 
    Grid _grid {};
	UnsortedList _openList;
	//std::vector<PathNode*> _openList; //TODO: Change to tree // TODO: Put it on the stack
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

    //Floyd Path Reconstruction
    void FloydPathReconstruction();
	bool Floyd_IsNeighbour(GridPos& inStart, GridPos& inEnd, bool& outIsDiagonal);
	std::vector<int> Floyd_GetPath(GridPos& inStart, GridPos& inEnd);
	void CreateFloydPath(WaypointList& inPath, const std::vector<int>& inPathIndices);
    bool Floyd_IsValidPosition(const int inStart, const int inEnd, bool& outIsDiagonal);
};

