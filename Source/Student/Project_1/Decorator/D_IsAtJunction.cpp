#include <pch.h>
#include "D_Delay.h"
#include "D_IsAtJunction.h"
#include "../GlobalInfo.h"

D_IsAtJunction::D_IsAtJunction()
{}

void D_IsAtJunction::on_enter()
{
    BehaviorNode::on_enter();
}

void D_IsAtJunction::on_update(float dt)
{
    BehaviorNode *child = children.front();
    auto& bb = agent->get_blackboard();

    if (bb.get_value<int>("Junction Index") != -1)
    {
        if(Vec3::Distance(agent->get_position(), GlobalInfo::junctionPoints[bb.get_value<int>("Junction Index")].position) > 15.0f)
        {
            bb.set_value("Junction Index", -1);
			bb.set_value<MovementDirection>("MovementDirection", MovementDirection::NONE);
            bb.set_value("ForwardDestinationSet", false);
        	on_failure();
            return;
        }

        child->tick(dt);

        // assume same status as child
        set_status(child->get_status());
        set_result(child->get_result());

        return;
    }

    for(int i = 0; i < GlobalInfo::junctionPoints.size(); ++i)
    {
        float distance = Vec3::Distance(agent->get_position(), GlobalInfo::junctionPoints[i].position);

        if(distance <= 20.0f)
        {
            bb.set_value("Junction Index", i);

            child->tick(dt);

            // assume same status as child
            set_status(child->get_status());
            set_result(child->get_result());

            return;
        }
    }

	bb.set_value("Junction Index", -1);
	on_failure();

}
