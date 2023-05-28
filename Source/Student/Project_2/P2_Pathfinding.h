#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "PathNode.h"
#include <vector>

constexpr int GRID_WIDTH = 40;
constexpr int GRID_HEIGHT = 40;

using Grid = std::array<std::array<PathNode*, GRID_WIDTH>, GRID_HEIGHT>;

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
	std::vector<PathNode*> _closedList; //TODO: Change to tree
    PathNode* _goalNode{ nullptr };
    PathNode* _parentNode{ nullptr };

	// Pathfinding Functions
    PathNode* GetCheapestNodeInOpenLost();
	void UpdateNodeAccessibleNeighbours(PathNode* inPathNode);
	void AddNeighbourToOpenList(PathNode* inPathNode);
	bool IsNodeInOpenList(PathNode* inPathNode);
	bool IsNodeInClosedList(PathNode* inPathNode);

    // Heuristic Distance Functions
	float manhattanDistance(const GridPos& inStart, const GridPos& inEnd);
	float chebyshevDistance(const GridPos& inStart, const GridPos& inEnd);
	float euclideanDistance(const GridPos& inStart, const GridPos& inEnd);
	float octileDistance(const GridPos& inStart, const GridPos& inEnd);
	float inconsistentHeuristic(const GridPos& inStart, const GridPos& inEnd);

};