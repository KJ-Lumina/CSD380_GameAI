#include <pch.h>
#include "L_GetAllNearbyBoid.h"
#include "../GlobalInfo.h"

L_GetAllNearbyBoid::L_GetAllNearbyBoid()
{}

void L_GetAllNearbyBoid::on_enter()
{
	BehaviorNode::on_leaf_enter();
}

void L_GetAllNearbyBoid::on_update(float dt)
{
	on_success();
    display_leaf_text();
}