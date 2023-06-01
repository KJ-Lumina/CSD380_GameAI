#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "PathNode.h"
#include <vector>
#include <queue>

constexpr int GRID_WIDTH = 40;
constexpr int GRID_HEIGHT = 40;
constexpr float SQRT_2 = 1.41421356237f;
constexpr float NODE_DIAGONAL_COST = SQRT_2;
constexpr float NODE_STRAIGHT_COST = 1.0f;

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


    Grid _grid;
    std::vector<PathNode*> _openList; //TODO: Change to tree
    PathNode* _goalNode{ nullptr };
    PathNode* _parentNode{ nullptr };
    Heuristic _heuristic{ Heuristic::MANHATTAN };
    float _weight = 1.0f;
    bool _debugColoring{ false };
	bool _singleStep = false;

	// Pathfinding Functions
    PathNode* GetCheapestNodeInOpenList();
    void UpdateAllNodeNeighbours();
	void UpdateNodeAccessibleNeighbours(PathNode* inPathNode);
	void AddAllNeighboursToOpenList(PathNode* inPathNode);

    //Create the Path Node Compare Function
    float CalculateHeuristic(GridPos inStart);
    void ResetGrid(int inWidth, int inHeight);
};