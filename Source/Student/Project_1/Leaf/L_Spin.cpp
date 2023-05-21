#include <pch.h>
#include "L_Spin.h"

L_Spin::L_Spin() 
{}

void L_Spin::on_enter()
{

	BehaviorNode::on_leaf_enter();
}

void L_Spin::on_update(float dt)
{
	if(spinTimer <= 0.0f)
	{
		spinTimer = 1.0f;
		on_success();
	}

	agent->set_yaw(agent->get_yaw() + (spinSpeed * dt));
	spinTimer -= dt;

    display_leaf_text();
}