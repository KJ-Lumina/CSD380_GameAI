#pragma once
#include <array>
#include "PathNode.h"

constexpr int GRID_WIDTH = 40;
constexpr int GRID_HEIGHT = 40;

using Grid = std::array<std::array<PathNode, GRID_WIDTH>, GRID_HEIGHT>;

class GridInfo
{
public:
	Grid grid;
};