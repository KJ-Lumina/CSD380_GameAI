#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "PathNode.h"
#include <vector>
#include <queue>

constexpr int GRID_WIDTH = 40;
constexpr int GRID_HEIGHT = 40;
constexpr float NODE_DIAGONAL_COST = 1.41421356237f;
constexpr float NODE_STRAIGHT_COST = 1.0f;

using Grid = std::array<std::array<PathNode*, GRID_WIDTH>, GRID_HEIGHT>;

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
	//std::priority_queue<PathNode*, std::vector<PathNode*>, PathNodeCompare> _openList;
	std::vector<PathNode*> _openList; //TODO: Change to tree
	std::vector<PathNode*> _closedList; //TODO: Change to tree
    PathNode* _goalNode{ nullptr };
    PathNode* _parentNode{ nullptr };

	// Pathfinding Functions
    PathNode* GetCheapestNodeInOpenList();
    void UpdateAllNodeNeighbours();
	void UpdateNodeAccessibleNeighbours(PathNode* inPathNode);
	void AddAllNeighboursToOpenList(PathNode* inPathNode);
	bool IsNodeInOpenList(PathNode* inPathNode);
	bool IsNodeInClosedList(PathNode* inPathNode);
    PathNode* GetNodeInOpenList(PathNode* inPathNode);
	PathNode* GetNodeInClosedList(PathNode* inPathNode);
	void RemoveNodeFromOpenList(PathNode* inPathNode);
	void RemoveNodeFromClosedList(PathNode* inPathNode);

    //Create the Path Node Compare Function

    void ResetGrid();

    // Heuristic Distance Functions
	float manhattanDistance(const GridPos& inStart, const GridPos& inEnd);
	float chebyshevDistance(const GridPos& inStart, const GridPos& inEnd);
	float euclideanDistance(const GridPos& inStart, const GridPos& inEnd);
	float octileDistance(const GridPos& inStart, const GridPos& inEnd);
	float inconsistentHeuristic(const GridPos& inStart, const GridPos& inEnd);

};