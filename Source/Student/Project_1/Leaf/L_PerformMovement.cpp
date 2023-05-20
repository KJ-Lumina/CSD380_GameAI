#include <pch.h>
#include "L_PerformMovement.h"
#include "../GlobalInfo.h"

L_PerformMovement::L_PerformMovement()
{}

void L_PerformMovement::on_enter()
{
    BehaviorNode::on_leaf_enter();
}

void L_PerformMovement::on_update(float dt)
{
	auto& bb = agent->get_blackboard();
	MovementDirection dir = bb.get_value<MovementDirection>("MovementDirection");

	switch (dir)
	{
	case MovementDirection::LEFT:
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
            bb.set_value("PossibleTurnLeft", false);
			bb.set_value("PossibleTurnRight", false);
            bb.set_value("ForwardDestinationSet", false);;
        	on_success();
        }
        else
        {
            Vec3 destination = agent->get_position() + agent->get_up_vector() * agent->get_movement_speed() * dt;
            agent->set_position(destination);
            agent->set_yaw(agent->get_yaw() + (turnSpeed * dt));
            turnTimer -= dt;
        }
        break;
	}

	case MovementDirection::RIGHT:
	{
        if (!timerSet)
        {
            targetAngle = agent->get_yaw() - (PI / 2.0f);;
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
            bb.set_value("PossibleTurnLeft", false);
            bb.set_value("PossibleTurnRight", false);
            bb.set_value("ForwardDestinationSet", false);
        	on_success();
        }
        else
        {
            Vec3 destination = agent->get_position() + agent->get_up_vector() * agent->get_movement_speed() * dt;
            agent->set_position(destination);
            agent->set_yaw(agent->get_yaw() - (turnSpeed * dt));
            turnTimer -= dt;
        }
		break;
	}
	case MovementDirection::FORWARD:
    {
		auto& bb = agent->get_blackboard();
		bb.set_value("ForwardDestinationSet", false);

        if (!bb.get_value<bool>("ForwardDestinationSet")) {
            forwardDestination = agent->get_position() + (agent->get_up_vector() * 10.0f);
            bb.set_value("ForwardDestinationSet", true);
        }

        if (agent->move_toward_point(forwardDestination, dt))
        {
            std::cout << "Reached" << std::endl;
            bb.set_value("ForwardDestinationSet", false);
            auto& bb = agent->get_blackboard();
            bb.set_value<MovementDirection>("MovementDirection", MovementDirection::NONE);
            on_success();
        }

		break;
	}

	default:
	{
	    Vec3 destination = agent->get_position() + agent->get_up_vector() * agent->get_movement_speed() * 3.0f * dt;
	    agent->set_position(destination);

	    on_success();
	}
	}

  /*  if(dir == MovementDirection::FORWARD)
    {
        const Vec3 destination = agent->get_position() + (agent->get_forward_vector() * 40.0f);

		if (agent->move_toward_point(destination, dt))
		{
			on_success();
		}
    }
    else
    {
        Vec3 destination = agent->get_position() + agent->get_up_vector() * agent->get_movement_speed() * dt;
        agent->set_position(destination);

        on_success();
    }*/

    display_leaf_text();
}