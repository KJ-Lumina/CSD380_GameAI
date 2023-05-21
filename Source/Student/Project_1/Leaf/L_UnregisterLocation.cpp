#include <pch.h>
#include "L_UnregisterLocation.h"
#include "../GlobalInfo.h"

L_UnregisterLocation::L_UnregisterLocation()
{}

void L_UnregisterLocation::on_enter()
{
	BehaviorNode::on_leaf_enter();
}

void L_UnregisterLocation::on_update(float dt)
{
	auto& bb = agent->get_blackboard();
	bb.set_value("TargetPosition", Vec3::Zero);

	on_success();
    display_leaf_text();
}