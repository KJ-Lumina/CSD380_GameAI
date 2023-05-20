#include <pch.h>
#include "L_VehicleRegister.h"
#include "..\GlobalInfo.h"

L_VehicleRegister::L_VehicleRegister()
{}

void L_VehicleRegister::on_enter()
{
	agent->set_scaling(Vec3(2.0f, 1.0f, 1.0f));
	agent->set_position(Vec3(7.5f, 0, 7.5f));
	agent->set_pitch(PI / 2.0f);
	GlobalInfo::vehicleAgentsID.push_back(agent->get_id());
	auto& bb = agent->get_blackboard();
	bb.set_value("Junction Index", -1);
	bb.set_value("PossibleTurnLeft", false);
	bb.set_value("PossibleTurnRight", false);
	bb.set_value("PossibleForward", false);
	bb.set_value<MovementDirection>("MovementDirection", MovementDirection::FORWARD);
	bb.set_value("ForwardDestinationSet", false);
	BehaviorNode::on_leaf_enter();
}

void L_VehicleRegister::on_update(float dt)
{
	on_success();
    display_leaf_text();
}
