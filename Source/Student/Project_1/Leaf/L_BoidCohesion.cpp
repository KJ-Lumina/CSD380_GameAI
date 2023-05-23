#include <pch.h>
#include "L_BoidCohesion.h"
#include "../GlobalInfo.h"

L_BoidCohesion::L_BoidCohesion()
{}

void L_BoidCohesion::on_enter()
{
	BehaviorNode::on_leaf_enter();
}

void L_BoidCohesion::on_update(float dt)
{
	on_success();
    display_leaf_text();
}