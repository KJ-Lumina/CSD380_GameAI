#include <pch.h>
#include "L_DigUnderground.h"

L_DigUnderground::L_DigUnderground()
{}

void L_DigUnderground::on_enter()
{
	BehaviorNode::on_leaf_enter();

	if(!isDigging)
	{
		DigDestination = agent->get_position() + DigDepth;
		isDigging = true;
	}
}

void L_DigUnderground::on_update(float dt)
{
	if (agent->move_toward_point(DigDestination, dt))
	{
		on_success();
		isDigging = false;
	}

    display_leaf_text();
}