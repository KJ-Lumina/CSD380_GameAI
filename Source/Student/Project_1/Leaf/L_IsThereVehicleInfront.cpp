#include <pch.h>
#include "L_IsThereVehicleInfront.h"
#include "../GlobalInfo.h"

L_IsThereVehicleInfront::L_IsThereVehicleInfront()
{}

void L_IsThereVehicleInfront::on_enter()
{
	BehaviorNode::on_leaf_enter();
}

void L_IsThereVehicleInfront::on_update(float dt)
{
	auto& bb = agent->get_blackboard();
	
	for(auto vehicleAgent : GlobalInfo::vehicleAgents)
	{
		float distance = Vec3::Distance(vehicleAgent->get_position(), agent->get_position());
		if (distance < 1.0f)
		{
			on_failure();
			return;
		}
	}

	on_success();
    display_leaf_text();
}