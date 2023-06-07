#pragma once
#include <Misc/NiceTypes.h>
#include <array>
#include <bitset>

enum NodeState
{
	NONE = 0,
	OPEN,
	CLOSED
};

class PathNode
{
public:
	PathNode* parent{ nullptr };
	GridPos gridPosition{0, 0 };
	int givenCost{ 0 };
	int finalCost{ 0 };
	NodeState nodeStates;
	char neighbours;

	void setNeighbor(int index, bool value) {
		if (value) {
			// Set the bit at the given index to 1.
			neighbours |= 1 << index;
		}
		else {
			// Set the bit at the given index to 0.
			neighbours &= ~(1 << index);
		}
	}

	bool getNeighbor(int index) {
		// Get the bit at the given index.
		return (neighbours >> index) & 1;
	}

	void Reset()
	{
		nodeStates = NodeState::NONE;
		givenCost = 0;
		finalCost = 0;
	}

	bool operator== (const PathNode& other) const
	{
		return gridPosition == other.gridPosition;
	}
};