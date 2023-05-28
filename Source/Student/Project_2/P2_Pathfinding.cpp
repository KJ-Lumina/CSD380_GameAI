#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"
#include <algorithm>

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have

    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */

	//Setting up the grid
	for (int i = 0; i < GRID_HEIGHT; i++)
	{
		for (int j = 0; j < GRID_WIDTH; j++)
		{
			_grid[i][j] = new PathNode();
			_grid[i][j]->gridPosition = { i, j };
		}
	}

	for (int i = 0; i < terrain->get_map_height(); i++)
	{
		for (int j = 0; j < terrain->get_map_width(); j++)
		{
            UpdateNodeAccessibleNeighbours(_grid[i][j]);

			// do something with terrain->is_wall(i, j)
		}
	}

    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

PathResult AStarPather::compute_path(PathRequest &request)
{
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */

    // Declaring the start and goal nodes
    GridPos start = terrain->get_grid_position(request.start);
    GridPos goal = terrain->get_grid_position(request.goal);
    // Assign the goal node
	_goalNode = _grid[goal.row][goal.col];

	// Find out which heuristic is selected and calculated it for all nodes based on terrain size
    switch(request.settings.heuristic)
    {

		case Heuristic::MANHATTAN:
		{
			for (int i = 0; i < terrain->get_map_width(); i++)
			{
				for (int j = 0; j < terrain->get_map_height(); j++)
				{
					GridPos current = { i, j };
					_grid[i][j].heuristicCost = manhattanDistance(current, goal);
				}
			}
            break;
		}

		case Heuristic::EUCLIDEAN:
		{
			//Euclidean
            for (int i = 0; i < terrain->get_map_width(); i++)
            {
                for (int j = 0; j < terrain->get_map_height(); j++)
                {
                    GridPos current = { i, j };
                    _grid[i][j].heuristicCost = euclideanDistance(current, goal);
                }
            }
            break;
		}
		case Heuristic::CHEBYSHEV:
		{
            for (int i = 0; i < terrain->get_map_width(); i++)
            {
                for (int j = 0; j < terrain->get_map_height(); j++)
                {
                    GridPos current = { i, j };
                    _grid[i][j].heuristicCost = chebyshevDistance(current, goal);
                }
            }
            break;
		}

		case Heuristic::OCTILE:
		{
			//Octile
            for (int i = 0; i < terrain->get_map_width(); i++)
            {
                for (int j = 0; j < terrain->get_map_height(); j++)
                {
                    GridPos current = { i, j };
                    _grid[i][j].heuristicCost = octileDistance(current, goal);
                }
            }
			break;
		}

		case Heuristic::INCONSISTENT:
        {
            for (int i = 0; i < terrain->get_map_width(); i++)
            {
                for (int j = 0; j < terrain->get_map_height(); j++)
                {
                    GridPos current = { i, j };
                    _grid[i][j].heuristicCost = inconsistentHeuristic(current, goal);
                }
            }
            break;
        }
    }

    if(request.newRequest)
    {
		//Clear the open and closed lists
        _openList.clear();
        _closedList.clear();

		//Pushing the start node onto the open list
        GridPos start = terrain->get_grid_position(request.start);
    }

    while (!_openList.empty())
    {
        _parentNode = GetCheapestNodeInOpenLost();
        if(_parentNode == _goalNode)
			return PathResult::COMPLETE;

		if (request.settings.singleStep)
		{
			return PathResult::PROCESSING;
		}
    }

	return PathResult::IMPOSSIBLE;

    
    // Just sample code, safe to delete

    terrain->set_color(start, Colors::Orange);
    terrain->set_color(goal, Colors::Orange);
    request.path.push_back(request.start);
    request.path.push_back(request.goal);
    return PathResult::COMPLETE;
}

/*************************************************************************
 *                      Helper A* FUNCTIONS
 *************************************************************************/
PathNode* AStarPather::GetCheapestNodeInOpenLost() //TODO: Optimize using a binary tree
{
	PathNode* cheapestNode = _openList.front();
	for (auto node : _openList)
	{
		if (node->finalCost < cheapestNode->finalCost)
		{
			cheapestNode = node;
		}
	}
	return cheapestNode;
}

void AStarPather::UpdateNodeAccessibleNeighbours(PathNode* inPathNode)
{
	//Get the neighbours of the current node
    int neighbourIndex = 0;
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
			if (i == 0 && j == 0)
				continue; //Skip the current node (itself)

			bool isWall = terrain->is_wall(GridPos{ inPathNode->gridPosition.row + i, inPathNode->gridPosition.col + j });
			bool isValid = terrain->is_valid_grid_position(GridPos{ inPathNode->gridPosition.row + i, inPathNode->gridPosition.col + j });
            if(!isWall && !isValid)
            {
                inPathNode->neighbours[neighbourIndex] = _grid[inPathNode->gridPosition.row + i][inPathNode->gridPosition.col + j];
            }

			neighbourIndex++;
        }
    }
}

//Add the neighbours of the node to the open list TODO: Add in the given cost of the node [Diagonal [Sqrt 2] would be cheaper than straight [1]]
void AStarPather::AddNeighbourToOpenList(PathNode* inPathNode)
{
    std::vector<int> neighboursIndex{ 1,3,4,6 };
	std::vector<int> diagonalNeighborsIndex = { 0,2,5,7 };
	std::vector<bool> isDiagonalNeighbourValid = { false, false, false, false };

	// Add the neighbours of the node to the open list (Top, Left, Right, Bottom)
    for(auto index : neighboursIndex)
    {
	    if(inPathNode->neighbours[index] != nullptr)
	    {
			_openList.push_back(inPathNode->neighbours[index]);
	    }
    }

	//Check if the diagonal neighbours are valid
    if(inPathNode->neighbours[1] != nullptr && inPathNode->neighbours[3])
        isDiagonalNeighbourValid[0] = true;

    if (inPathNode->neighbours[1] != nullptr && inPathNode->neighbours[4])
        isDiagonalNeighbourValid[2] = true;

    if (inPathNode->neighbours[6] != nullptr && inPathNode->neighbours[3])
        isDiagonalNeighbourValid[5] = true;

    if (inPathNode->neighbours[6] != nullptr && inPathNode->neighbours[4])
        isDiagonalNeighbourValid[7] = true;

	// Add the valid diagonal neighbours to the open list
	for (int i = 0; i < isDiagonalNeighbourValid.size(); i++)
	{
		if (isDiagonalNeighbourValid[i])
		{
			_openList.push_back(inPathNode->neighbours[diagonalNeighborsIndex[i]]);
		}
	}
}

//Check if the node is in the open list
bool AStarPather::IsNodeInOpenList(PathNode* inPathNode) // TODO: Optimize using a binary tree
{
    std::find(_openList.begin(), _openList.end(), inPathNode);
}

//Check if the node is in the closed list
bool AStarPather::IsNodeInClosedList(PathNode* inPathNode) // TODO: Optimize using a binary tree
{
	std::find(_closedList.begin(), _closedList.end(), inPathNode);
}

/*************************************************************************
 *                      HEURISTIC FUNCTIONS
 *************************************************************************/
float AStarPather::manhattanDistance(const GridPos& inStart, const GridPos& inEnd)
{
	return std::abs(inStart.row - inEnd.row) + std::abs(inStart.col - inEnd.col);
}

float AStarPather::chebyshevDistance(const GridPos& inStart, const GridPos& inEnd)
{
	return std::max(std::abs(inStart.row - inEnd.row), std::abs(inStart.col - inEnd.col));
}

float AStarPather::euclideanDistance(const GridPos& inStart, const GridPos& inEnd)
{
	return std::sqrt(std::pow(inStart.row - inEnd.row, 2) + std::pow(inStart.col - inEnd.col, 2));
}

float AStarPather::octileDistance(const GridPos& inStart, const GridPos& inEnd)
{
	float a = std::min(std::abs(inStart.row - inEnd.row), std::abs(inStart.col - inEnd.col)) * std::sqrt(2);
    float b = std::max(std::abs(inStart.row - inEnd.row), std::abs(inStart.col - inEnd.col));
	float c = std::min(std::abs(inStart.row - inEnd.row), std::abs(inStart.col - inEnd.col));

	return a + b - c;
}

float AStarPather::inconsistentHeuristic(const GridPos& inStart, const GridPos& inEnd)
{
    return ((inStart.row + inStart.col % 2) > 0) ? euclideanDistance(inStart, inEnd) : 0.0f;
}