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
	for (int i = 0; i < GRID_HEIGHT; ++i)
	{
		for (int j = 0; j < GRID_WIDTH; ++j)
		{
			_grid[i][j].gridPosition = { i, j };
		}
	}

    Callback cb = std::bind(&AStarPather::UpdateAllNodeNeighbours, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

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
        ResetGrid(terrain->get_map_width(), terrain->get_map_height());

		GridPos start = terrain->get_grid_position(request.start);
		GridPos goal = terrain->get_grid_position(request.goal);

		//Clear the open and closed lists
	    _openList.clear();

        _heuristic = request.settings.heuristic; // Setting the current heuristic for this request
		_debugColoring = request.settings.debugColoring; // Setting the current debug coloring for this request
		_weight = request.settings.weight; // Setting the current weight for this request
		_singleStep = request.settings.singleStep; // Setting the current single step for this request

		_goalNode = &_grid[goal.row][goal.col];

		//Pushing the start node onto the open list
		PathNode& startNode = _grid[start.row][start.col];
        startNode.parent = nullptr;
        startNode.SetOpenList(true);
		_openList.push_back(&_grid[start.row][start.col]);
        std::push_heap(_openList.begin(), _openList.end(), PathNodeCompare());
    }

    while (!_openList.empty())
    {
		std::pop_heap(_openList.begin(), _openList.end(), PathNodeCompare());
        _parentNode = _openList.back();
        _openList.pop_back();

        if (_parentNode == _goalNode) {

            PathNode* node = _parentNode->parent;

            while(node != nullptr)
            {
                request.path.emplace_front(terrain->get_world_position(node->gridPosition));
                node = node->parent;
            }

            request.path.emplace_back(request.goal);
            return PathResult::COMPLETE;
        }

        _parentNode->SetOpenList(false);
		_parentNode->SetClosedList(true);

        if (_debugColoring)
			terrain->set_color(_parentNode->gridPosition, Colors::Yellow);

        AddAllNeighboursToOpenList(_parentNode);

		if (_singleStep)
		{
			return PathResult::PROCESSING;
		}
    }

	return PathResult::IMPOSSIBLE;
}

/*************************************************************************
 *                      Helper A* FUNCTIONS
 *************************************************************************/
void AStarPather::UpdateAllNodeNeighbours()
{
    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
            // Does a check to see if the neighbours and wall/in-accessible and preupdate them before pathfinding computation
            UpdateNodeAccessibleNeighbours(&_grid[i][j]);
        }
    }
}

void AStarPather::UpdateNodeAccessibleNeighbours(PathNode* inPathNode)
{
	//Get the neighbours of the current node
    int neighbourIndex = 0;
    for (int i = -1; i < 2; ++i)
    {
        for (int j = -1; j < 2; ++j)
        {
			if (i == 0 && j == 0)
				continue; //Skip the current node (itself)

            if (!terrain->is_valid_grid_position(GridPos{ inPathNode->gridPosition.row + i, inPathNode->gridPosition.col + j })) {
				inPathNode->neighbours[neighbourIndex] = false;
                neighbourIndex++;
                continue; //Skip the current node (itself
            }

			inPathNode->neighbours[neighbourIndex] = (!terrain->is_wall(GridPos{ inPathNode->gridPosition.row + i, inPathNode->gridPosition.col + j })) ? true : false;

            ++neighbourIndex;
        }
    }
}

//Add the neighbours of the node to the open list
void AStarPather::AddAllNeighboursToOpenList(PathNode* inPathNode)
{
    std::array<bool, 8> isNeighbourValid = inPathNode->neighbours;

    //Check if the diagonal neighbours are valid
    if(!inPathNode->neighbours[1])
    {
        isNeighbourValid[0] = false;
        isNeighbourValid[2] = false;
    }

    if(!inPathNode->neighbours[3])
    {
        isNeighbourValid[0] = false;
        isNeighbourValid[5] = false;
    }

    if (!inPathNode->neighbours[4])
    {
        isNeighbourValid[2] = false;
        isNeighbourValid[7] = false;
    }

    if (!inPathNode->neighbours[6])
    {
        isNeighbourValid[5] = false;
        isNeighbourValid[7] = false;
    }

	// Add the neighbours of the node to the open list (Top, Left, Right, Bottom)
    
	for(size_t index = 0; index < inPathNode->neighbours.size(); ++index)
	{
        if (!isNeighbourValid[index])
	        continue;

        GridPos gp { inPathNode->gridPosition.row + neighbourOffsets[index].row, inPathNode->gridPosition.col + neighbourOffsets[index].col };
        PathNode* neighbour = &_grid[gp.row][gp.col];

        // Adding the given cost of the current node to the neighbour PLUS the straight cost
        const float newGivenCost = inPathNode->givenCost + neighbourCost[index];
        const float newFinalCost = newGivenCost + (CalculateHeuristic(neighbour->gridPosition) * _weight);

        if (!neighbour->IsOnOpenList() && !neighbour->IsOnClosedList())
        {
	        neighbour->parent = inPathNode;
	        neighbour->givenCost = newGivenCost;
	        neighbour->finalCost = newFinalCost;
	        neighbour->SetOpenList(true);

	        _openList.push_back(neighbour);
	        std::push_heap(_openList.begin(), _openList.end(), PathNodeCompare());

	        if (_debugColoring)
		        terrain->set_color(neighbour->gridPosition, Colors::Blue);
        }
        else
        {
	        if (neighbour->IsOnOpenList()) // If the node is in the open list
	        {
		        // Remove the node from the open list if the current neighbour is cheaper than the one that is on it.
		        if (newFinalCost < neighbour->finalCost)
		        {
			        neighbour->SetClosedList(false);

			        neighbour->parent = inPathNode;
			        neighbour->givenCost = newGivenCost;
			        neighbour->finalCost = newFinalCost;
			        neighbour->SetOpenList(true);

			        std::make_heap(_openList.begin(), _openList.end(), PathNodeCompare());

			        if(_debugColoring)
				        terrain->set_color(neighbour->gridPosition, Colors::Blue);
		        }

	        }
	        else if (neighbour->IsOnClosedList()) // If the node is in the closed list
	        {
		        if (newFinalCost < neighbour->finalCost)
		        {
			        neighbour->SetClosedList(false);

			        neighbour->parent = inPathNode;
			        neighbour->givenCost = newGivenCost;
			        neighbour->finalCost = newFinalCost;
			        neighbour->SetOpenList(true);
			        if (_debugColoring)
				        terrain->set_color(neighbour->gridPosition, Colors::Blue);
			        _openList.push_back(neighbour);
			        std::push_heap(_openList.begin(), _openList.end(), PathNodeCompare());
		        }
	        }
        }
	}
}

float AStarPather::CalculateHeuristic(GridPos& inStart)
{
    const float diffX = std::fabsf(static_cast<float>(inStart.row - _goalNode->gridPosition.row));
    const float diffY = std::fabsf(static_cast<float>(inStart.col - _goalNode->gridPosition.col));

	switch(_heuristic)
	{
		case Heuristic::MANHATTAN:
		{
            return diffX + diffY;
		}

		case Heuristic::EUCLIDEAN:
        {
            return std::sqrtf(std::powf(diffX, 2) + std::powf(diffY, 2));
        }

		case Heuristic::CHEBYSHEV:
        {
           return std::fmax(diffX, diffY);
        }

		case Heuristic::OCTILE:
        {
            return std::fmin(diffX, diffY) * SQRT_2 + std::fmax(diffX, diffY) - std::fmin(diffX, diffY);
        }

		case Heuristic::INCONSISTENT:
        {
            return ((inStart.row + inStart.col % 2) > 0) ? std::sqrtf(std::powf(diffX, 2) + std::powf(diffY, 2)) : 0.0f;
        }
	}

    return 0.0f;
}

void AStarPather::ResetGrid(const int inWidth, const int inHeight)
{
    for (int i = 0; i < inHeight; ++i)
    {
        for (int j = 0; j < inWidth; ++j)
        {
            _grid[i][j].Reset();
        }
    }
}
