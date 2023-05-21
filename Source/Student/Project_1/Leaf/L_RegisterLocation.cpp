#include <pch.h>
#include "L_RegisterLocation.h"
#include "../GlobalInfo.h"

L_RegisterLocation::L_RegisterLocation()
{}

void L_RegisterLocation::on_enter()
{
	BehaviorNode::on_leaf_enter();
}

void L_RegisterLocation::on_update(float dt)
{
	std::vector<size_t> PotentialRoute;
	for(size_t i = 0; i < GlobalInfo::grassPatchPosition.size(); ++i)
	{
		if (Vec3::Distance(GlobalInfo::grassPatchPosition[i], agent->get_position()) < 10.0f)
			continue;

		PotentialRoute.push_back(i);
	}

	int rand = RNG::range(0, static_cast<int>(PotentialRoute.size() - 1));

	auto& bb = agent->get_blackboard();
	bb.set_value("TargetPosition", GlobalInfo::grassPatchPosition[rand]);

	on_success();
    display_leaf_text();
}