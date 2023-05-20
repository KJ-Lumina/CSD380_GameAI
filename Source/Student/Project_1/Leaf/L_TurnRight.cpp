#include <pch.h>
#include "L_TurnRight.h"

L_TurnRight::L_TurnRight()
{}

void L_TurnRight::on_enter()
{
    targetAngle = agent->get_yaw() - (PI / 2.0f);;
	turnTimer = 0.0f;
	timerSet = false;
    BehaviorNode::on_leaf_enter();
}

void L_TurnRight::on_update(float dt)
{
    if (!timerSet)
    {
		float angleDiff = agent->get_yaw() - targetAngle;
        turnTimer = (angleDiff / turnSpeed);
        timerSet = true;
    }

    if(turnTimer <= 0)
    {
        on_success();
    }
	else
    {
        Vec3 destination = agent->get_position() + agent->get_up_vector() * agent->get_movement_speed() * dt;
        agent->set_position(destination);
        agent->set_yaw(agent->get_yaw() - (turnSpeed * dt));
		turnTimer -= dt;
    }

    display_leaf_text();
}