#include <pch.h>
#include "L_AppearAboveGround.h"

L_AppearAboveGround::L_AppearAboveGround()
{}

void L_AppearAboveGround::on_enter()
{
	if (LocationPreSet == false) {
		auto& bb = agent->get_blackboard();
		Vec3 position = bb.get_value<Vec3>("DigLocation");
		agent->set_position(Vec3(position.x, agent->get_position().y, position.z));
		LocationPreSet = true;
	}
	BehaviorNode::on_leaf_enter();
}

void L_AppearAboveGround::on_update(float dt)
{
	auto& bb = agent->get_blackboard();
	Vec3 position = bb.get_value<Vec3>("DigLocation");

	if(agent->move_toward_point(position, dt))
	{
		LocationPreSet = false;
		on_success();
	}

    display_leaf_text();
}