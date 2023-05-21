#pragma once
#include "Blackboard.h"

enum class MovementDirection
{
	NONE,
	FORWARD,
	LEFT,
	RIGHT,
	COUNT
};

struct JunctionOrientationPassasge
{
	Vec3 orientation { 0.0f, 0.0f, 0.0f };
	MovementDirection turnDirection{ MovementDirection::COUNT };
	bool isStraightPossible = true;

	Vec3 CheckOrientationAlignment() const
	{
		float x = fabs(orientation.x);
		float z = fabs(orientation.z);

		return (x > z) ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(0.0f, 0.0f, 1.0f);
	}

	Vec3 GetDirectionVector() const
	{
		return (orientation * -1.0f) * 15.0f;
	}
};

struct Junction
{
	Vec3 position{ 0.0f, 0.0f, 0.0f };
	std::vector<JunctionOrientationPassasge> orientationPassages;
};

class GlobalInfo
{
public:
	inline static Blackboard globalBlackboard;
	inline static std::vector<Junction> junctionPoints;
	inline static std::vector<BehaviorAgent*> vehicleAgents;
	inline static std::vector<Vec3> grassPatchPosition;
};