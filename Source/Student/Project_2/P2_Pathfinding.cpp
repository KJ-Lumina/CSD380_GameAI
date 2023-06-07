#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"
#include <algorithm>

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return true;
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

    Callback floydcb = std::bind(&AStarPather::FloydPathReconstruction, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, floydcb);

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
		if (request.settings.method != Method::FLOYD_WARSHALL){

            ResetGrid();

            GridPos start = terrain->get_grid_position(request.start);
            GridPos goal = terrain->get_grid_position(request.goal);

            //Clear the open and closed lists
            _openList.gridSize = 0;

            _heuristic = request.settings.heuristic; // Setting the current heuristic for this request
            _debugColoring = request.settings.debugColoring; // Setting the current debug coloring for this request
            _weight = request.settings.weight; // Setting the current weight for this request
            _singleStep = request.settings.singleStep; // Setting the current single step for this request

            _goalNode = &_grid[goal.row][goal.col];

            //Pushing the start node onto the open list
            PathNode& startNode = _grid[start.row][start.col];
            startNode.parent = nullptr;
            startNode.nodeStates = NodeState::OPEN;

            _openList.Push(&_grid[start.row][start.col]);

		}
		else
        {
		    GridPos start = terrain->get_grid_position(request.start);
		    GridPos goal = terrain->get_grid_position(request.goal);

		    const std::vector<int> path = Floyd_GetPath(start, goal);

		    if (path.empty())
		    {
		        return PathResult::IMPOSSIBLE;
		    }

		    CreateFloydPath(request.path, path);

		    return PathResult::COMPLETE;
		}
    }

    while (_openList.gridSize > 0)
    {
        _parentNode = _openList.FindCheapestNodeAndPop();

        if (_parentNode == _goalNode) {

            const PathNode* node = _parentNode->parent;

            while(node != nullptr)
            {
                request.path.emplace_front(terrain->get_world_position(node->gridPosition));
                node = node->parent;
            }

            request.path.emplace_back(request.goal);

        	if (request.settings.rubberBanding && request.settings.smoothing)
		    {
		        RubberBandPath(request.path);
		        AddBackNodes(request.path);
		        SmoothPath(request.path);
		    }
            else if(request.settings.rubberBanding)
            {
                RubberBandPath(request.path);
            }
            else if(request.settings.smoothing)
            {
                SmoothPath(request.path);
            }

            return PathResult::COMPLETE;
        }

        _parentNode->nodeStates = NodeState::CLOSED;

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
                inPathNode->neighbours &= ~(1 << neighbourIndex);
                neighbourIndex++;
                continue; //Skip the current node (itself
            }

			const bool isWall = terrain->is_wall(GridPos{ inPathNode->gridPosition.row + i, inPathNode->gridPosition.col + j });

			// Set the bit at the given index to 1.
            inPathNode->setNeighbor(neighbourIndex, (!terrain->is_wall(GridPos{ inPathNode->gridPosition.row + i, inPathNode->gridPosition.col + j })) ? true : false);

            ++neighbourIndex;
        }
    }

    if (!(inPathNode->neighbours >> 1 & 1))
    {
        // Set the bit at the given index to 0.
        inPathNode->neighbours &= ~(1 << 0);
        inPathNode->neighbours &= ~(1 << 2);
    }

    if (!(inPathNode->neighbours >> 3 & 1))
    {
        inPathNode->neighbours &= ~(1 << 0);
        inPathNode->neighbours &= ~(1 << 5);
    }

    if (!(inPathNode->neighbours >> 4 & 1))
    {
        inPathNode->neighbours &= ~(1 << 2);
        inPathNode->neighbours &= ~(1 << 7);
    }

    if (!(inPathNode->neighbours >> 6 & 1))
    {
        inPathNode->neighbours &= ~(1 << 5);
        inPathNode->neighbours &= ~(1 << 7);
    }
}

//Add the neighbours of the node to the open list
void AStarPather::AddAllNeighboursToOpenList(PathNode* inPathNode)
{
	// Add the neighbours of the node to the open list (Top, Left, Right, Bottom)
    
	for(int index = 0; index < 8; ++index)
	{
        if (inPathNode->neighbours >> index & 1) {

            GridPos gp{ inPathNode->gridPosition.row + neighbourOffsets[index].row, inPathNode->gridPosition.col + neighbourOffsets[index].col };
            PathNode* neighbour = &_grid[gp.row][gp.col];

            // Adding the given cost of the current node to the neighbour PLUS the straight cost
            const int newGivenCost = inPathNode->givenCost + neighbourCost[index];
            const int newFinalCost = newGivenCost + static_cast<int>(CalculateHeuristic(neighbour->gridPosition, _heuristic, _goalNode->gridPosition) * _weight);

            if (neighbour->nodeStates == 0)
            {
                neighbour->parent = inPathNode;
                neighbour->givenCost = newGivenCost;
                neighbour->finalCost = newFinalCost;
                neighbour->nodeStates = NodeState::OPEN;

                _openList.Push(neighbour);

                if (_debugColoring)
                    terrain->set_color(neighbour->gridPosition, Colors::Blue);
            }
            else if (neighbour->nodeStates != 0 && newFinalCost < neighbour->finalCost)
            {
                neighbour->parent = inPathNode;
                neighbour->givenCost = newGivenCost;
                neighbour->finalCost = newFinalCost;
            }
        }
	}
}

void AStarPather::RubberBandPath(WaypointList& inPath)
{
	for (int i = 1; i < inPath.size() - 1; ++i)
	{
        auto it = inPath.end();
		std::advance(it, -(i + 1));

		auto itPrev = it;
		std::advance(itPrev, -1);

		auto itNext = it;
		std::advance(itNext, 1);

        GridPos itGridPos = terrain->get_grid_position(*it);
        GridPos itPrevGridPos = terrain->get_grid_position(*itPrev);
		GridPos itNextGridPos = terrain->get_grid_position(*itNext);

        if(IsNodeDeletable(itGridPos,itPrevGridPos, itNextGridPos))
        {
            inPath.erase(it);
            --i;
        }
	}
}

bool AStarPather::IsNodeDeletable(const GridPos& inCurrent, const GridPos& inStart, const GridPos& inEnd)
{
    int diffX = inStart.row - inEnd.row;
    int diffY = inStart.col - inEnd.col;

    if(diffY == 0)
    {
		if (diffX < 0)
		{
			for (int i = inStart.row; i <= inEnd.row; ++i)
			{
				if (terrain->is_wall({ i, inCurrent.col }))
					return false;
			}
		}
		else if(diffX > 0)
		{
            for (int i = inEnd.row; i <= inStart.row; ++i)
            {
                if (terrain->is_wall({ i, inCurrent.col }))
                    return false;
            }
		}
	}
	else if(diffY > 0)
    {
	    if(diffX == 0)
	    {
            for (int j = inEnd.col; j <= inStart.col; ++j)
            {
                if (terrain->is_wall({ inCurrent.row, j }))
                    return false;
            }
	    }
		else if(diffX > 0)
	    {
            for (int i = inEnd.row; i <= inStart.row; ++i) {
                for (int j = inEnd.col; j <= inStart.col; ++j)
                {
                    if (terrain->is_wall({ i, j }))
                        return false;
                }
            }
	    }
		else if(diffX < 0)
	    {
			for (int i = inStart.row; i <= inEnd.row; ++i) {
                for (int j = inEnd.col; j <= inStart.col; ++j)
                {
                    if (terrain->is_wall({ i, j }))
                        return false;
                }
            }
	    }
    }
    else if (diffY < 0)
    {
        if (diffX == 0)
        {
            for (int j = inStart.col; j <= inEnd.col; ++j)
            {
                if (terrain->is_wall({ inStart.row, j }))
                    return false;
            }
        }
        else if (diffX > 0)
        {
            for (int i = inEnd.row; i <= inStart.row; ++i) {
                for (int j = inStart.col; j <= inEnd.col; ++j)
                {
                    if (terrain->is_wall({ i, j }))
                        return false;
                }
            }
        }
        else if (diffX < 0)
        {
            for (int i = inStart.row; i <= inEnd.row; ++i) {
                for (int j = inStart.col; j <= inEnd.col; ++j)
                {
                    if (terrain->is_wall({ i, j }))
                        return false;
                }
            }
        }
    }

    return true;
}

void AStarPather::SmoothPath(WaypointList& inPath)
{
    std::vector<Vec3> inputPath;
    std::list<Vec3> resultPath;

    for(auto& path : inPath)
    {
        inputPath.emplace_back(path);
    }

    for(int i = 0; i < inputPath.size() - 1; ++i)
    {
        resultPath.emplace_back(inputPath[i]);

        Vec3 pt1 = (i == 0) ? inputPath[0] : inputPath[i - 1];
        Vec3 pt2 = inputPath[i];
        Vec3 pt3 = inputPath[i + 1];
        Vec3 pt4 = (i == inputPath.size() - 2) ? inputPath[i + 1] : inputPath[i + 2];

        float t = 0.25f;

        for (int j = 0; j < 3; ++j) {
            Vec3 result = Vec3::CatmullRom(pt1, pt2, pt3, pt4, t);
            t += 0.25f;
            resultPath.emplace_back(result);
        }
    }

    resultPath.emplace_back(inputPath[inputPath.size() - 1]); // Inserting the last node

    inPath = resultPath;
}

void AStarPather::AddBackNodes(WaypointList& inPath)
{
	const float distance = Vec3::Distance(terrain->get_world_position(0, 1), terrain->get_world_position(0, 0)) * 1.5f;

    for(int i = 0; i < inPath.size() - 1; ++i)
    {
        WaypointList::iterator it_1 = inPath.begin();
        std::advance(it_1, i);
        WaypointList::iterator it_2 = std::next(it_1, 1);
        Vec3 pt1 = *it_1;
        Vec3 pt2 = *it_2;

        while (Vec3::Distance(pt1 ,pt2) > distance)
        {
            //Get Middle Node
            Vec3 direction{ pt2 - pt1 };
            Vec3 midPos{ pt1 + (direction / 2.0f) };
            inPath.insert(it_2, midPos);
            it_2 = std::next(it_1, 1);
            pt2 = *it_2;
        }
    }
}

void AStarPather::ResetGrid()
{
    for (auto& row : _grid)
    {
	    for (auto &node : row)
	    {
            node.Reset();
	    }
    }
}

void AStarPather::FloydPathReconstruction()
{
    //Initialize the values
    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; ++j) {
            distance[i][j] = INF;
            previous[i][j] = -1;
        }
    }

    //Update the costs
    for (int u = 0; u < V; ++u) {
        for (int v = 0; v < V; ++v) {
            if (u == v) {
                distance[u][v] = 0; // Distance from a node to itself is 0
                previous[u][v] = v;
            }

			// If there is an edge between u and v, then distance is 1
            bool isDiagonalNeighbour = false;
            if(Floyd_IsValidPosition(u, v, isDiagonalNeighbour))
            {
	            if(isDiagonalNeighbour)
	            {
                    distance[u][v] = NODE_DIAGONAL_COST;
	            }
            	else
	            {
                    distance[u][v] = NODE_STRAIGHT_COST;
	            }
                previous[u][v] = u;
            }
        }
    }

    for (int k = 0; k < V; k++) {
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                if (distance[i][j] > distance[i][k] + distance[k][j]) {
                    distance[i][j] = distance[i][k] + distance[k][j];
                    previous[i][j] = previous[k][j];
                }
            }
        }
    }
}

bool AStarPather::Floyd_IsNeighbour(GridPos& inStart, GridPos& inEnd, bool& outIsDiagonal)
{
    int x1 = inStart.row;
    int y1 = inStart.col;
    int x2 = inEnd.row;
    int y2 = inEnd.col;

    // Check if they are direct neighbors
    bool directNeighbor = ((x1 == x2 && abs(y1 - y2) == 1) || (y1 == y2 && abs(x1 - x2) == 1));

    // Check if they are diagonal neighbors
    bool diagonalNeighbor = (abs(x1 - x2) == 1 && abs(y1 - y2) == 1);

    outIsDiagonal = diagonalNeighbor;

    return directNeighbor || diagonalNeighbor;
}

std::vector<int> AStarPather::Floyd_GetPath(GridPos& inStart, GridPos& inEnd)
{
    int start = inStart.row * 40 + inStart.col;
    int end = inEnd.row * 40 + inEnd.col;

    if (previous[start][end] == -1) {
        return std::vector<int>();  // Empty path
    }

    std::vector<int> path;
    path.push_back(end);
    int u = start, v = end;
    while (u != v) {
        v = previous[u][v];
        path.insert(path.begin(), v);
    }
    return path;
}

void AStarPather::CreateFloydPath(WaypointList& inPath, const std::vector<int>& inPathIndices)
{
    for(auto& index : inPathIndices){
        GridPos pos{ index / 40, index % 40 };
        inPath.emplace_back(terrain->get_world_position(pos));
    }
}

bool AStarPather::Floyd_IsValidPosition(const int inStart, const int inEnd, bool& outIsDiagonal)
{
    const int x1 = inStart / GRID_HEIGHT;
    const int y1 = inStart % GRID_WIDTH;
    const int x2 = inEnd / GRID_HEIGHT;
    const int y2 = inEnd % GRID_WIDTH;

	GridPos start{ x1, y1 };
	GridPos end{ x2, y2 };

	bool isNeighbour = Floyd_IsNeighbour(start, end, outIsDiagonal);

	if (!isNeighbour)
	{
		return false;
	}

	if (!terrain->is_valid_grid_position(end) || terrain->is_wall(end))
	{
		return false;
	}

    // Check for Diagonal Correctness
    if(outIsDiagonal)
    {
        if (x2 > x1)
        {
            if (y2 > y1)
            {
                GridPos bottom{ x1, y1 + 1 };
                GridPos right{ x1 + 1, y1 };
                if (!terrain->is_valid_grid_position(bottom) || terrain->is_wall(bottom) || !terrain->is_valid_grid_position(right) || terrain->is_wall(right))
                {
                    return false;
                }
            }
            else
            {
                GridPos top{ x1, y1 - 1 };
                GridPos right{ x1 + 1, y1 };
                if (!terrain->is_valid_grid_position(top) || terrain->is_wall(top) || !terrain->is_valid_grid_position(right) || terrain->is_wall(right))
                {
                    return false;
                }
            }
        }
        else
        {
            if (y2 > y1)
            {
                GridPos bottom{ x1, y1 + 1 };
                GridPos left{ x1 - 1, y1 };
                if (!terrain->is_valid_grid_position(bottom) || terrain->is_wall(bottom) || !terrain->is_valid_grid_position(left) || terrain->is_wall(left))
                {
                    return false;
                }
            }
            else
            {
                GridPos top{ x1, y1 - 1 };
                GridPos left{ x1 - 1, y1 };
                if (!terrain->is_valid_grid_position(top) || terrain->is_wall(top) || !terrain->is_valid_grid_position(left) || terrain->is_wall(left))
                {
                    return false;
                }
            }
        }
    }

    return true;
}

/*************************************************************************
 *                      Unsorted List FUNCTIONS
 *************************************************************************/

void UnsortedList::Push(PathNode* inPathNode) {
    _list[gridSize] = inPathNode;
    ++gridSize;
}

PathNode* UnsortedList::FindCheapestNodeAndPop()
{
    if (gridSize == 0) {
        throw std::out_of_range("List is empty");
    }

    int minCost = INT_MAX;
    int minIndex = -1;

    for (int i = 0; i < gridSize; ++i) {
        if (_list[i]->finalCost < minCost) {
            minCost = _list[i]->finalCost;
            minIndex = i;
        }
    }

    PathNode* cheapestNode = _list[minIndex];
    // Move every element after the cheapest node one step towards the beginning of the list
    std::move(_list.begin() + minIndex + 1, _list.begin() + gridSize, _list.begin() + minIndex);
    --gridSize;

    return cheapestNode;
}
