#include <pch.h>
#include "D_IsMovementRegistered.h"
#include "../GlobalInfo.h"

D_IsMovementRegistered::D_IsMovementRegistered()
{}

void D_IsMovementRegistered::on_enter()
{
    BehaviorNode::on_enter();
}

void D_IsMovementRegistered::on_update(float dt)
{
    BehaviorNode *child = children.front();
	auto& bb = agent->get_blackboard();

	if (checked == false)
	{
	
		MovementDirection dir = bb.get_value<MovementDirection>("MovementDirection");
		if (dir != MovementDirection::NONE)
		{
			on_failure();
			return;
		}

		checked = true;
	}

	child->tick(dt);

	if (child->succeeded())
	{
		checked = false;
	}

	MovementDirection dir = bb.get_value<MovementDirection>("MovementDirection");

    // assume same status as child
    set_status(child->get_status());
    set_result(child->get_result());
}
