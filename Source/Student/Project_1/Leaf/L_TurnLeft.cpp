#include <pch.h>
#include "L_TurnLeft.h"
#include "../GlobalInfo.h"

L_TurnLeft::L_TurnLeft()
{}

void L_TurnLeft::on_enter()
{
    BehaviorNode::on_leaf_enter();
}

void L_TurnLeft::on_update(float dt)
{
    if (!timerSet)
    {
        targetAngle = agent->get_yaw() + (PI / 2.0f);;
        float angleDiff = agent->get_yaw() - targetAngle;
        turnTimer = fabs((angleDiff / turnSpeed));
        timerSet = true;
    }

    if (turnTimer <= 0)
    {
        agent->set_yaw(targetAngle);
        timerSet = false;
		auto& bb = agent->get_blackboard();
		bb.set_value<MovementDirection>("MovementDirection", MovementDirection::NONE);
        on_success();
    }
    else
    {
        Vec3 destination = agent->get_position() + agent->get_up_vector() * agent->get_movement_speed() * dt;
        agent->set_position(destination);
        agent->set_yaw(agent->get_yaw() + (turnSpeed * dt));
        turnTimer -= dt;
    }

    display_leaf_text();
}