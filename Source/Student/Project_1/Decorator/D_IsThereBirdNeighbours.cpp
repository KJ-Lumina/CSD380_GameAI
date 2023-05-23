
#include <pch.h>
#include "D_IsThereBirdNeighbours.h"

D_IsThereBirdNeighbours::D_IsThereBirdNeighbours() 
{}

void D_IsThereBirdNeighbours::on_enter()
{
    BehaviorNode::on_enter();
}

void D_IsThereBirdNeighbours::on_update(float dt)
{
	auto& bb = agent->get_blackboard();
    auto& organizer = agent->get_organizer()->get_all_agents();

    int count = 0;

    for(auto& agent : organizer)
    {
		if (Vec3::Distance(agent->get_position(), agent->get_position()) < 10.0f)
		{
			count++;
		}
    }

	bb.set_value("BirdNeighbours", count);

    BehaviorNode *child = children.front();

    child->tick(dt);

    // assume same status as child
    set_status(child->get_status());
    set_result(child->get_result());
    
}
