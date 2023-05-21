#include <pch.h>
#include "L_LookTowardCamera.h"
#include "../GlobalInfo.h"

L_LookTowardCamera::L_LookTowardCamera()
{}

void L_LookTowardCamera::on_enter()
{
	BehaviorNode::on_leaf_enter();
	agent->set_yaw(-PI / 2.0f);
}

void L_LookTowardCamera::on_update(float dt)
{
	on_success();
    display_leaf_text();
}