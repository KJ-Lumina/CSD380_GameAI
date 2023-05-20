#include <pch.h>
#include "L_WaitForTwoSecond.h"

L_WaitForTwoSecond::L_WaitForTwoSecond()
{}

void L_WaitForTwoSecond::on_enter()
{
	timer = 2.0f;

    BehaviorNode::on_leaf_enter();
}

void L_WaitForTwoSecond::on_update(float dt)
{
    timer -= dt;

	if (timer <= 0.0f)
	{
		on_success();
	}

    display_leaf_text();
}