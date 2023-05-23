#include <pch.h>
#include "L_BoidSeparation.h"
#include "../GlobalInfo.h"

L_BoidSeparation::L_BoidSeparation()
{}

void L_BoidSeparation::on_enter()
{
	BehaviorNode::on_leaf_enter();
}

void L_BoidSeparation::on_update(float dt)
{
	on_success();
    display_leaf_text();
}