#pragma once
#include <Misc/NiceTypes.h>
#include <array>

enum class NodeType
{
	NONE,
	OPEN,
	CLOSED,
	COUNT
};

class PathNode
{
public:
	std::array<PathNode*, 8> neighbours;
	PathNode* parent{ nullptr };
	GridPos gridPosition{0, 0 };
	float givenCost{ 0.0f };
	float heuristicCost{ 0.0f };
	float finalCost{ 0.0f };

	bool operator== (const PathNode& other) const
	{
		return gridPosition == other.gridPosition;
	}

	//NodeType nodeType = NodeType::NONE; // Switch to char and use a bitfield to save memory

private:
};