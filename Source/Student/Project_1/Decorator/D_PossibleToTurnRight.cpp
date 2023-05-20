#include <pch.h>
#include "D_Delay.h"
#include "D_PossibleToTurnRight.h"
#include "../GlobalInfo.h"

D_PossibleToTurnRight::D_PossibleToTurnRight()
{}

void D_PossibleToTurnRight::on_enter()
{
    BehaviorNode::on_enter();
}

void D_PossibleToTurnRight::on_update(float dt)
{
	BehaviorNode* child = children.front();

	auto& bb = agent->get_blackboard();
	int junctionIndex = bb.get_value<int>("Junction Index");

	if (junctionIndex == -1)
	{
		std::cout << "Junction Index is -1" << std::endl;
		on_failure();
		return;
	}

	//MovementDirection turnDirection = bb.get_value<MovementDirection>("Turn Direction");

	//if (turnDirection == MovementDirection::RIGHT)
	//{
	//	child->tick(dt);

	//	//if (child->succeeded())
	//	//{
	//	//	bb.set_value("Turn Direction", MovementDirection::NONE);
	//	//}

	//	// assume same status as child
	//	set_status(child->get_status());
	//	set_result(child->get_result());

	//	return;
	//}

	Junction& junction = GlobalInfo::junctionPoints[junctionIndex];

	for (JunctionOrientationPassasge& passages : junction.orientationPassages)
	{
		Vec3 passagePositionAlignment = (junction.position + passages.GetDirectionVector()) * passages.CheckOrientationAlignment();

		Vec3 agentAlignment = agent->get_position() * passages.CheckOrientationAlignment();

		Vec3 diffVec = passagePositionAlignment - agentAlignment;

		//Find Magnitude of Diff Vec
		float magnitude = sqrt(diffVec.x * diffVec.x + diffVec.y * diffVec.y + diffVec.z * diffVec.z);;
		if (magnitude > 0.1f)
		{
			continue;
		}

		//Check if the vectors are similar
		float dotProduct = agent->get_up_vector().x * passages.orientation.x + agent->get_up_vector().y * passages.orientation.y + agent->get_up_vector().z * passages.orientation.z;
		float magnitudeA = sqrt(agent->get_up_vector().x * agent->get_up_vector().x + agent->get_up_vector().y * agent->get_up_vector().y + agent->get_up_vector().z * agent->get_up_vector().z);
		float magnitudeB = sqrt(passages.orientation.x * passages.orientation.x + passages.orientation.y * passages.orientation.y + passages.orientation.z * passages.orientation.z);

		float cosTheta = dotProduct / (magnitudeA * magnitudeB);

		if (cosTheta > 1.0f - FLT_EPSILON && passages.turnDirection == MovementDirection::RIGHT)
		{
			child->tick(dt);

			// assume same status as child
			set_status(child->get_status());
			set_result(child->get_result());

			return;
		}
	}
}
