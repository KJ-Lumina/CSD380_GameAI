#include <pch.h>
#include "L_SelectMovement.h"
#include "..\GlobalInfo.h"

L_SelectMovement::L_SelectMovement()
{}

void L_SelectMovement::on_enter()
{
	BehaviorNode::on_leaf_enter();
	auto& bb = agent->get_blackboard();

	bool left = bb.get_value<bool>("PossibleTurnLeft");
	bool right = bb.get_value<bool>("PossibleTurnRight");
	bool forward = bb.get_value<bool>("PossibleForward");

	std::vector<MovementDirection> possibleTurns;
	if (forward)
		possibleTurns.push_back(MovementDirection::FORWARD);
	if (left)
		possibleTurns.push_back( MovementDirection::LEFT);
	if (right)
		possibleTurns.push_back(MovementDirection::RIGHT);

	if (possibleTurns.empty()) {
		bb.set_value<MovementDirection>("MovementDirection", MovementDirection::FORWARD); // This should never happen, but just in case :
	}
	else {
		int random = RNG::range(0, static_cast<int>(possibleTurns.size() - 1));
		MovementDirection dir = possibleTurns[random];
		bb.set_value<MovementDirection>("MovementDirection", dir);
	}

	bb.set_value("PossibleForward", false);
	bb.set_value("PossibleTurnLeft", false);
	bb.set_value("PossibleTurnRight", false);
}

void L_SelectMovement::on_update(float dt)
{
	on_success();
    display_leaf_text();
}