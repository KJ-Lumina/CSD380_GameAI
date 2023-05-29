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

    Callback cb = std::bind(&AStarPather::UpdateAllNodeNeighbours, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

	//std::cout << "Map Width: " << terrain->get_map_width() << std::endl;
	//std::cout << "Map Height: " << terrain->get_map_height() << std::endl;

	//for (int i = 0; i < terrain->get_map_height(); i++)
	//{
	//	for (int j = 0; j < terrain->get_map_width(); j++)
	//	{
 //           // Does a check to see if the neighbours and wall/in-accessible and preupdate them before pathfinding computation
	//		UpdateNodeAccessibleNeighbours(_grid[i][j]); 
	//	}
	//}

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
    if (request.newRequest)
    {
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
						_grid[i][j]->heuristicCost = manhattanDistance(current, goal);
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
		                _grid[i][j]->heuristicCost = euclideanDistance(current, goal);
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
		                _grid[i][j]->heuristicCost = chebyshevDistance(current, goal);
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
		                _grid[i][j]->heuristicCost = octileDistance(current, goal);
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
		                _grid[i][j]->heuristicCost = inconsistentHeuristic(current, goal);
		            }
		        }
		        break;
		    }
		}


		//Clear the open and closed lists
	    _openList.clear();
	    _closedList.clear();

		//Pushing the start node onto the open list
        _grid[start.row][start.col]->parent = nullptr;
		_openList.push_back(_grid[start.row][start.col]);
    }

    while (!_openList.empty())
    {
        _parentNode = GetCheapestNodeInOpenList();
        if (_parentNode == _goalNode) {

            PathNode* node = _parentNode->parent;


            while(node != nullptr)
            {
				request.path.push_front(terrain->get_world_position(node->gridPosition));
                node = node->parent;
            }

            request.path.push_back(request.goal);
            return PathResult::COMPLETE;
        }

        terrain->set_color(_parentNode->gridPosition, Colors::Yellow);
		_closedList.push_back(_parentNode);
        AddAllNeighboursToOpenList(_parentNode);

		if (request.settings.singleStep)
		{
			return PathResult::PROCESSING;
		}
    }

	return PathResult::IMPOSSIBLE;

    
    // Just sample code, safe to delete

    //terrain->set_color(start, Colors::Orange);
    //terrain->set_color(goal, Colors::Orange);
    //request.path.push_back(request.start);
    //request.path.push_back(request.goal);
    //return PathResult::COMPLETE;
}

/*************************************************************************
 *                      Helper A* FUNCTIONS
 *************************************************************************/
PathNode* AStarPather::GetCheapestNodeInOpenList() //TODO: Optimize using a binary tree
{
	PathNode* cheapestNode = _openList.front();
	int c_index = 0;

    for(int i = 0; i < _openList.size(); ++i)
    {
        if (_openList[i]->finalCost < cheapestNode->finalCost)
        {
            cheapestNode = _openList[i];
			c_index = i;
        }
    }

    _openList.erase(_openList.begin() + c_index);

	return cheapestNode;
}

void AStarPather::UpdateAllNodeNeighbours()
{
    for (int i = 0; i < terrain->get_map_height(); i++)
    {
        for (int j = 0; j < terrain->get_map_width(); j++)
        {
            // Does a check to see if the neighbours and wall/in-accessible and preupdate them before pathfinding computation
            UpdateNodeAccessibleNeighbours(_grid[i][j]);
        }
    }
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

            bool isValid = terrain->is_valid_grid_position(GridPos{ inPathNode->gridPosition.row + i, inPathNode->gridPosition.col + j });

            if (!isValid) {
				inPathNode->neighbours[neighbourIndex] = nullptr;
                neighbourIndex++;
                continue; //Skip the current node (itself
            }

			bool isWall = terrain->is_wall(GridPos{ inPathNode->gridPosition.row + i, inPathNode->gridPosition.col + j });
			
            if(!isWall)
            {
                inPathNode->neighbours[neighbourIndex] = _grid[inPathNode->gridPosition.row + i][inPathNode->gridPosition.col + j];
            }
        	else
            {
                inPathNode->neighbours[neighbourIndex] = nullptr;
            }

            neighbourIndex++;
        }
    }
}

//Add the neighbours of the node to the open list
void AStarPather::AddAllNeighboursToOpenList(PathNode* inPathNode)
{
    std::vector<int> neighboursIndex{ 1,3,4,6 };
	std::vector<int> diagonalNeighborsIndex = { 0,2,5,7 };
	std::vector<bool> isDiagonalNeighbourValid = { false, false, false, false };

	// Add the neighbours of the node to the open list (Top, Left, Right, Bottom)
    for(auto index : neighboursIndex)
    {
	    if(inPathNode->neighbours[index] != nullptr)
	    {
            PathNode* neighbour = inPathNode->neighbours[index];
           
            // Adding the given cost of the current node to the neighbour PLUS the straight cost
			float newGivenCost = inPathNode->givenCost + NODE_STRAIGHT_COST;
			float newFinalCost = newGivenCost + neighbour->heuristicCost;

			bool isNodeInOpenList = IsNodeInOpenList(neighbour);
			bool isNodeInClosedList = IsNodeInClosedList(neighbour);

			if (!isNodeInOpenList && !isNodeInClosedList)
            {
                terrain->set_color(neighbour->gridPosition, Colors::Blue);
                neighbour->parent = inPathNode;
                neighbour->givenCost = newGivenCost;
                neighbour->finalCost = newFinalCost;
				_openList.push_back(neighbour);
            }
	    	else
			{
				if (isNodeInOpenList) // If the node is in the open list
                {
                    PathNode* node = GetNodeInOpenList(neighbour);

                    // Remove the node from the open list if the current neighbour is cheaper than the one that is on it.
					if (newFinalCost < node->finalCost)
					{
						RemoveNodeFromOpenList(node);
						RemoveNodeFromClosedList(node);
                        neighbour->parent = inPathNode;
                        neighbour->givenCost = newGivenCost;
                        neighbour->finalCost = newFinalCost;
                        terrain->set_color(neighbour->gridPosition, Colors::Blue);
                        _openList.push_back(neighbour);
					}

				}
				else if (isNodeInClosedList) // If the node is in the closed list
				{
                    PathNode* node = GetNodeInClosedList(neighbour);

                    if (newFinalCost < node->finalCost)
                    {
                        RemoveNodeFromOpenList(node);
                        RemoveNodeFromClosedList(node);
                        neighbour->parent = inPathNode;
                        neighbour->givenCost = newGivenCost;
                        neighbour->finalCost = newFinalCost;
                        terrain->set_color(neighbour->gridPosition, Colors::Blue);
                        _openList.push_back(neighbour);
                    }
				}
			}
	    }
    }

	//Check if the diagonal neighbours are valid
    if(inPathNode->neighbours[1] != nullptr && inPathNode->neighbours[3] != nullptr)
        isDiagonalNeighbourValid[0] = true;

    if (inPathNode->neighbours[1] != nullptr && inPathNode->neighbours[4] != nullptr)
        isDiagonalNeighbourValid[1] = true;

    if (inPathNode->neighbours[6] != nullptr && inPathNode->neighbours[3] != nullptr)
        isDiagonalNeighbourValid[2] = true;

    if (inPathNode->neighbours[6] != nullptr && inPathNode->neighbours[4] != nullptr)
        isDiagonalNeighbourValid[3] = true;

	// Add the valid diagonal neighbours to the open list
	for (size_t i = 0; i < isDiagonalNeighbourValid.size(); i++)
	{
        uint8_t index = diagonalNeighborsIndex[i];

		if (isDiagonalNeighbourValid[i] && inPathNode->neighbours[index] != nullptr)
		{
            PathNode* neighbour = inPathNode->neighbours[index];

            // Adding the given cost of the current node to the neighbour PLUS the straight cost
            float newGivenCost = inPathNode->givenCost + NODE_DIAGONAL_COST;
            float newFinalCost = newGivenCost + neighbour->heuristicCost;

            bool isNodeInOpenList = IsNodeInOpenList(neighbour);
            bool isNodeInClosedList = IsNodeInClosedList(neighbour);

            if (!isNodeInOpenList && !isNodeInClosedList)
            {
                neighbour->parent = inPathNode;
                neighbour->givenCost = newGivenCost;
                neighbour->finalCost = newFinalCost;
                terrain->set_color(neighbour->gridPosition, Colors::Blue);
                _openList.push_back(neighbour);
            }
            else
            {
                if (isNodeInOpenList) // If the node is in the open list
                {
                    PathNode* node = GetNodeInOpenList(neighbour);

                    // Remove the node from the open list if the current neighbour is cheaper than the one that is on it.
                    if (newFinalCost < node->finalCost)
                    {
                        RemoveNodeFromOpenList(node);
                        RemoveNodeFromClosedList(node);
                        neighbour->parent = inPathNode;
                        neighbour->givenCost = newGivenCost;
                        neighbour->finalCost = newFinalCost;
                        terrain->set_color(neighbour->gridPosition, Colors::Blue);
                        _openList.push_back(neighbour);
                    }

                }
                else if (isNodeInClosedList) // If the node is in the closed list
                {
                    PathNode* node = GetNodeInClosedList(neighbour);

                    if (newFinalCost < node->finalCost)
                    {
                        RemoveNodeFromOpenList(node);
                        RemoveNodeFromClosedList(node);
                        neighbour->parent = inPathNode;
                        neighbour->givenCost = newGivenCost;
                        neighbour->finalCost = newFinalCost;
                        terrain->set_color(neighbour->gridPosition, Colors::Blue);
                        _openList.push_back(neighbour);
                    }
                }
            }
		}
	}
}

//Check if the node is in the open list
bool AStarPather::IsNodeInOpenList(PathNode* inPathNode) // TODO: Optimize using a binary tree
{
    auto it = std::find(_openList.begin(), _openList.end(), inPathNode);

    if (it == _openList.end())
        return false;

    return true;
}

//Check if the node is in the closed list
bool AStarPather::IsNodeInClosedList(PathNode* inPathNode) // TODO: Optimize using a binary tree
{
	auto it = std::find(_closedList.begin(), _closedList.end(), inPathNode);

    if (it == _closedList.end())
        return false;

    return true;
}

PathNode* AStarPather::GetNodeInOpenList(PathNode* inPathNode)
{
    auto it = std::find(_openList.begin(), _openList.end(), inPathNode);

    if (it == _openList.end())
        return nullptr;

    size_t index = it - _openList.begin();
    return _openList[index];
}

PathNode* AStarPather::GetNodeInClosedList(PathNode* inPathNode)
{
    auto it = std::find(_closedList.begin(), _closedList.end(), inPathNode);

    if (it == _closedList.end())
        return nullptr;

    size_t index = it - _closedList.begin();
    return _closedList[index];
}

void AStarPather::RemoveNodeFromOpenList(PathNode* inPathNode)
{
    auto it = std::find(_openList.begin(), _openList.end(), inPathNode);

    if (it == _openList.end())
        return;

    size_t index = it - _openList.begin();
    _openList.erase(_openList.begin() + index);
}

void AStarPather::RemoveNodeFromClosedList(PathNode* inPathNode)
{
    auto it = std::find(_closedList.begin(), _closedList.end(), inPathNode);

    if (it == _closedList.end())
        return;

    size_t index = it - _closedList.begin();
    _closedList.erase(_closedList.begin() + index);
}

/*************************************************************************
 *                      HEURISTIC FUNCTIONS
 *************************************************************************/
float AStarPather::manhattanDistance(const GridPos& inStart, const GridPos& inEnd)
{
    return static_cast<float>(std::abs(inStart.row - inEnd.row) + std::abs(inStart.col - inEnd.col));
}

float AStarPather::chebyshevDistance(const GridPos& inStart, const GridPos& inEnd)
{
    return static_cast<float>(std::max(std::abs(inStart.row - inEnd.row), std::abs(inStart.col - inEnd.col)));
}

float AStarPather::euclideanDistance(const GridPos& inStart, const GridPos& inEnd)
{
    return static_cast<float>(std::sqrt(std::pow(inStart.row - inEnd.row, 2) + static_cast<double>(std::pow(inStart.col - inEnd.col, 2))));
}

float AStarPather::octileDistance(const GridPos& inStart, const GridPos& inEnd)
{
    const float a = static_cast<float>(std::fmin(std::abs(inStart.row - inEnd.row), std::abs(inStart.col - inEnd.col))) * static_cast<float>(std::sqrt(2));
    const float b = static_cast<float>(std::fmax(std::abs(inStart.row - inEnd.row), std::abs(inStart.col - inEnd.col)));
	const float c = static_cast<float>(std::fmin(std::abs(inStart.row - inEnd.row), std::abs(inStart.col - inEnd.col)));

	return a + b - c;
}

float AStarPather::inconsistentHeuristic(const GridPos& inStart, const GridPos& inEnd)
{
    return ((inStart.row + inStart.col % 2) > 0) ? euclideanDistance(inStart, inEnd) : 0.0f;
}