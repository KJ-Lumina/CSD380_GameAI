#include <pch.h>
#include "L_MoveTowardsPosition.h"

L_MoveTowardsPosition::L_MoveTowardsPosition() : targetPosition{ Vec3::Zero }
{}

void L_MoveTowardsPosition::on_enter()
{
    //timer = RNG::range(1.0f, 2.0f);

	

    BehaviorNode::on_leaf_enter();
}

void L_MoveTowardsPosition::on_update(float dt)
{
    //timer -= dt;

    agent->set_position(targetPosition);

    on_success();

    display_leaf_text();
}