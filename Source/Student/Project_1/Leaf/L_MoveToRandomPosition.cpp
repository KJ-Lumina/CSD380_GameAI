#include <pch.h>
#include "L_MoveToRandomPosition.h"
#include "Agent/BehaviorAgent.h"

void L_MoveToRandomPosition::on_enter()
{
    // set animation, speed, etc

    targetPoint = RNG::world_position();
    //agent->look_at_point(targetPoint);

	BehaviorNode::on_leaf_enter();
}

void L_MoveToRandomPosition::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    //::cout << agent->get_organizer()->get_all_agents().size() << std::endl;

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
