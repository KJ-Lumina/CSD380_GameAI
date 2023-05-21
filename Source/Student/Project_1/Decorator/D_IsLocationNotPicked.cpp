#include <pch.h>
#include "D_IsLocationNotPicked.h"
#include "../GlobalInfo.h"

D_IsLocationNotPicked::D_IsLocationNotPicked()
{}

void D_IsLocationNotPicked::on_enter()
{
    BehaviorNode::on_enter();
}

void D_IsLocationNotPicked::on_update(float dt)
{
    BehaviorNode *child = children.front();
	auto& bb = agent->get_blackboard();

    if(bb.get_value<Vec3>("TargetPosition") == Vec3::Zero)
    {
		child->tick(dt);

        // assume same status as child
        set_status(child->get_status());
        set_result(child->get_result());
    }
    else
    {
		on_failure();
    }
}
