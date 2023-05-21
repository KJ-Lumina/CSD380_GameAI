#include <pch.h>
#include "L_MoveTowardsPosition.h"

L_MoveTowardsPosition::L_MoveTowardsPosition() : targetPosition{ Vec3::Zero }
{}

void L_MoveTowardsPosition::on_enter()
{
	targetPosition = agent->get_blackboard().get_value<Vec3>("TargetPosition");

    BehaviorNode::on_leaf_enter();
}

void L_MoveTowardsPosition::on_update(float dt)
{
	if (agent->move_toward_point(targetPosition, dt))
	{
		on_success();
	}

    display_leaf_text();
}