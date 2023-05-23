#include <pch.h>
#include "L_UpdateFlock.h"
#include "../FlockingInfo.h"

L_UpdateFlock::L_UpdateFlock()
{}

void L_UpdateFlock::on_enter()
{
	std::vector<Boid*> nearbyBoids;

	for (Boid& boid : FlockingInfo::allBoids)
	{
		if (boid.agentId == agent->get_id())
			continue; // skip self (don't add self to nearby boids)

		if (Vec3::Distance(boid.position, agent->get_position()) < m_DetectionRadius) // 
			nearbyBoids.push_back(&boid);
	}

	auto& bb = agent->get_blackboard();
	bb.set_value("NearbyBoids", nearbyBoids);

	BehaviorNode::on_leaf_enter();
}

void L_UpdateFlock::on_update(float dt)
{
	auto& bb = agent->get_blackboard();
	std::vector<Boid*> nearbyBoids = bb.get_value<std::vector<Boid*>>("NearbyBoids");
	Boid* myBoid = bb.get_value<Boid*>("MyBoid");

	const Vec3 separationResult = FlockingInfo::ComputeSeparation(myBoid, nearbyBoids, m_SeparationRadius, m_SeparationWeight);
	const Vec3 cohesionResult = FlockingInfo::ComputeCohesion(myBoid, nearbyBoids, m_CohesionRadius, m_CohesionWeight);
	const Vec3 alignmentResult = FlockingInfo::ComputeAlignment(myBoid, nearbyBoids, m_AlignmentWeight);
	const Vec3 wanderResult = FlockingInfo::ComputeWander(myBoid, m_WanderRadius, myBoid->wanderDistance, myBoid->wanderJitter, m_WanderWeight);

	myBoid->acceleration += separationResult;
	myBoid->acceleration += cohesionResult;
	myBoid->acceleration += alignmentResult;
	myBoid->acceleration += wanderResult;

	myBoid->velocity += myBoid->acceleration * dt;

	//Limit the speed of the boid
	if (myBoid->velocity.Length() > myBoid->maxSpeed)
	{
		myBoid->velocity.Normalize(myBoid->velocity);
		myBoid->velocity *= myBoid->maxSpeed;
	}

	myBoid->position += myBoid->velocity * dt;

	//Set rotation of the agent to face the velocity direction
	agent->set_yaw(atan2f(myBoid->velocity.x, myBoid->velocity.z));

	//Loop around the world (X-Axis)
	if (myBoid->position.x > 100.0f)
		myBoid->position.x = 0.0f;
	else if (myBoid->position.x < 0.0f)
		myBoid->position.x = 100.0f;

	//Loop around the world (Z-Axis)
	if (myBoid->position.z > 100.0f)
		myBoid->position.z = 0.0f;
	else if (myBoid->position.z < 0.0f)
		myBoid->position.z = 100.0f;

	agent->set_position(myBoid->position);

	myBoid->acceleration = Vec3::Zero; // Reset each acceleration cycle to 0

	bb.set_value<Boid*>("MyBoid", myBoid); //Update boid

	on_success();
    display_leaf_text();
}