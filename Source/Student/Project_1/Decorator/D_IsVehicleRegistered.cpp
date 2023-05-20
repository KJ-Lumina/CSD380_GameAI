#include <pch.h>
#include "D_IsVehicleRegistered.h"
#include "../GlobalInfo.h"

D_IsVehicleRegistered::D_IsVehicleRegistered() 
{}

void D_IsVehicleRegistered::on_enter()
{
    BehaviorNode::on_enter();
}

void D_IsVehicleRegistered::on_update(float dt)
{
    BehaviorNode* child = children.front();
	bool isRegistered = std::find(GlobalInfo::vehicleAgentsID.begin(), GlobalInfo::vehicleAgentsID.end(), agent->get_id()) != GlobalInfo::vehicleAgentsID.end();

	if (isRegistered)
	{
		on_failure();
	}
	else
	{
		child->tick(dt);

		// assume same status as child
		set_status(child->get_status());
		set_result(child->get_result());
	}
}
