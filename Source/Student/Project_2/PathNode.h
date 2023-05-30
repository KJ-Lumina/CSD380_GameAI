#pragma once
#include <Misc/NiceTypes.h>
#include <array>




class PathNode
{
public:
	std::array<bool, 8> neighbours;
	PathNode* parent{ nullptr };
	GridPos gridPosition{0, 0 };
	float givenCost{ 0.0f };
	float finalCost{ 0.0f };
	bool isOnOpenList{ false };
	bool isOnClosedList{ false };

	void Reset(int x, int y)
	{
		parent = nullptr;
		isOnOpenList = false;
		isOnClosedList = false;
		givenCost = 0.0f;
		finalCost = 0.0f;
		gridPosition = { x, y };
	}

	bool operator== (const PathNode& other) const
	{
		return gridPosition == other.gridPosition;
	}

private:
};