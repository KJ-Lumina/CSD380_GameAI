#pragma once
#include <Misc/NiceTypes.h>
#include <array>
#include <bitset>

class PathNode
{
public:
	PathNode* parent{ nullptr };
	GridPos gridPosition{0, 0 };
	float givenCost{ 0.0f };
	float finalCost{ 0.0f };
	std::bitset<2> nodeStates;
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
		nodeStates.reset();
		//givenCost = 0.0f;
		finalCost = 0.0f;
	}

	void SetOpenList(bool value)
	{
		nodeStates.set(0, value);
	}

	void SetClosedList(bool value)
	{
		nodeStates.set(1, value);
	}

	bool IsOnOpenList() const
	{
		return nodeStates.test(0);
	}

	bool IsOnClosedList() const
	{
		return nodeStates.test(1);
	}

	bool operator== (const PathNode& other) const
	{
		return gridPosition == other.gridPosition;
	}
};