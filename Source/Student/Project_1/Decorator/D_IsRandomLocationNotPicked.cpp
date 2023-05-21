#include <pch.h>
#include "D_IsRandomLocationNotPicked.h"
#include "../GlobalInfo.h"

D_IsRandomLocationNotPicked::D_IsRandomLocationNotPicked()
{}

void D_IsRandomLocationNotPicked::on_enter()
{
    BehaviorNode::on_enter();
}

void D_IsRandomLocationNotPicked::on_update(float dt)
{
    BehaviorNode *child = children.front();
	auto& bb = agent->get_blackboard();

    if(bb.get_value<Vec3>("DigLocation") == Vec3::Zero)
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
