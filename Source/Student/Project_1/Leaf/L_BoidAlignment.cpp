#include <pch.h>
#include "L_BoidAlignment.h"
#include "../GlobalInfo.h"

L_BoidAlignment::L_BoidAlignment()
{}

void L_BoidAlignment::on_enter()
{
	BehaviorNode::on_leaf_enter();
}

void L_BoidAlignment::on_update(float dt)
{
	on_success();
    display_leaf_text();
}