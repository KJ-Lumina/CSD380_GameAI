#include <pch.h>
#include "L_UnregisterRandomPatchLocation.h"
#include "../GlobalInfo.h"

L_UnregisterRandomPatchLocation::L_UnregisterRandomPatchLocation()
{}

void L_UnregisterRandomPatchLocation::on_enter()
{
	auto& bb = agent->get_blackboard();
	bb.set_value("DigLocation", Vec3(0.0f, 0.0f, 0.0f));

	BehaviorNode::on_leaf_enter();
}

void L_UnregisterRandomPatchLocation::on_update(float dt)
{
	on_success();
    display_leaf_text();
}