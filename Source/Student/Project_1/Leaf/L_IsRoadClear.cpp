#include <pch.h>
#include "L_IsRoadClear.h"

L_IsRoadClear::L_IsRoadClear()
{}

void L_IsRoadClear::on_enter()
{
    //timer = RNG::range(1.0f, 2.0f);

    BehaviorNode::on_leaf_enter();
}

void L_IsRoadClear::on_update(float dt)
{
    //timer -= dt;

    on_success();

    display_leaf_text();
}