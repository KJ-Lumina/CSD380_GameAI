#include <pch.h>
#include "L_WaitForOneSecond.h"

L_WaitForOneSecond::L_WaitForOneSecond()
{}

void L_WaitForOneSecond::on_enter()
{
	timer = 1.0f;

    BehaviorNode::on_leaf_enter();
}

void L_WaitForOneSecond::on_update(float dt)
{
    timer -= dt;

	if (timer <= 0.0f)
	{
		on_success();
	}

    display_leaf_text();
}