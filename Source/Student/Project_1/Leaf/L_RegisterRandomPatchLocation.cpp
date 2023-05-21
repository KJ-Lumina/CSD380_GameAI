#include <pch.h>
#include "L_RegisterRandomPatchLocation.h"
#include "../GlobalInfo.h"

L_RegisterRandomPatchLocation::L_RegisterRandomPatchLocation() 
{}

void L_RegisterRandomPatchLocation::on_enter()
{
	std::vector<size_t> PotentialRoute;
	for (size_t i = 0; i < GlobalInfo::grassPatchPosition.size(); ++i)
	{
		if (Vec3::Distance(GlobalInfo::grassPatchPosition[i], agent->get_position()) < 10.0f)
			continue;

		PotentialRoute.push_back(i);
	}

	int rand = RNG::range(0, static_cast<int>(PotentialRoute.size() - 1));

	auto& bb = agent->get_blackboard();

	float randAngle = RNG::range(0.0f, 360.0f * (PI / 180.0f)); // In Radians

	Vec3 randVector = Vec3(cos(randAngle), 0.0f, sin(randAngle)) * RNG::range(0.0f, 5.0f);

	bb.set_value("DigLocation", GlobalInfo::grassPatchPosition[rand] + randVector);

	BehaviorNode::on_leaf_enter();
}

void L_RegisterRandomPatchLocation::on_update(float dt)
{
	on_success();

    display_leaf_text();
}