#pragma once
#include <Misc/NiceTypes.h>
#include <array>
#include <bitset>



class PathNode
{
public:
	std::array<bool, 8> neighbours;
	PathNode* parent{ nullptr };
	GridPos gridPosition{0, 0 };
	float givenCost{ 0.0f };
	float finalCost{ 0.0f };
	std::bitset<2> nodeStates;

	void Reset(int x, int y)
	{
		parent = nullptr;
		nodeStates.reset();
		givenCost = 0.0f;
		finalCost = 0.0f;
		gridPosition = { x, y };
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



private:
};