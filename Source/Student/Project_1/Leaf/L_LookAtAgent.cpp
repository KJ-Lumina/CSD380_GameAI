#include <pch.h>
#include "L_LookAtAgent.h"

L_LookAtAgent::L_LookAtAgent() : targetAgent(Vec3::Zero)
{}

void L_LookAtAgent::on_enter()
{
    //timer = RNG::range(1.0f, 2.0f);

    BehaviorNode::on_leaf_enter();
}

void L_LookAtAgent::on_update(float dt)
{
    //timer -= dt;

    on_success();

    display_leaf_text();
}