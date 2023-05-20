#include <pch.h>
#include "L_RegisterRightTurn.h"
#include "../GlobalInfo.h"

L_RegisterRightTurn::L_RegisterRightTurn()
{}

void L_RegisterRightTurn::on_enter()
{
	BehaviorNode::on_leaf_enter();
}

void L_RegisterRightTurn::on_update(float dt)
{
	auto& bb = agent->get_blackboard();
	int junctionIndex = bb.get_value<int>("Junction Index");

	if (junctionIndex == -1)
	{
		std::cout << "Junction Index is -1" << std::endl;
		on_failure();
		return;
	}

	Junction& junction = GlobalInfo::junctionPoints[junctionIndex];

	for (JunctionOrientationPassasge& passages : junction.orientationPassages)
	{
		Vec3 passagePositionAlignment = (junction.position + passages.GetDirectionVector()) * passages.CheckOrientationAlignment();

		Vec3 agentAlignment = agent->get_position() * passages.CheckOrientationAlignment();

		Vec3 diffVec = passagePositionAlignment - agentAlignment;

		//Find Magnitude of Diff Vec
		float magnitude = sqrt(diffVec.x * diffVec.x + diffVec.y * diffVec.y + diffVec.z * diffVec.z);
		if (magnitude > 0.7f)
		{
			continue;
		}

		//Check if the vectors are similar
		float dotProduct = agent->get_up_vector().x * passages.orientation.x + agent->get_up_vector().y * passages.orientation.y + agent->get_up_vector().z * passages.orientation.z;
		float magnitudeA = sqrt(agent->get_up_vector().x * agent->get_up_vector().x + agent->get_up_vector().y * agent->get_up_vector().y + agent->get_up_vector().z * agent->get_up_vector().z);
		float magnitudeB = sqrt(passages.orientation.x * passages.orientation.x + passages.orientation.y * passages.orientation.y + passages.orientation.z * passages.orientation.z);

		float cosTheta = dotProduct / (magnitudeA * magnitudeB);

		if (cosTheta > 1.0f - 0.01f && passages.turnDirection == MovementDirection::RIGHT)
		{
			auto& bb = agent->get_blackboard();
			bb.set_value("PossibleTurnRight", true);
			on_success();
			return;
		}
	}

	on_success();
	display_leaf_text();
}