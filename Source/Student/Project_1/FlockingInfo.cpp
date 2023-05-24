#include <pch.h>
#include "FlockingInfo.h"

// Helper Functions to limit the vector by a max value
Vec3 LimitVector(Vec3 inVector, float inMax)
{
	if (inVector.Length() > inMax)
	{
		inVector.Normalize(inVector);
		inVector *= inMax;
	}
	return inVector;
}

// Helper Functions to calculate the seek vector
Vec3 Seek(const Boid* myBoid, Vec3 inSeekedVector)
{
	Vec3 direction = inSeekedVector - myBoid->position;
	direction.Normalize(direction);
	direction *= myBoid->maxSpeed;

	Vec3 steerVector = direction - myBoid->velocity;
	steerVector = LimitVector(steerVector, myBoid->maxForce);

	return steerVector;
}

Vec3 FlockingInfo::ComputeSeparation(const Boid* myBoid, const std::vector<Boid*>& nearbyBoids, float separationRadius, float separationWeight)
{
	Vec3 separationResult{ Vec3::Zero };
	int separationCount = 0;
	for (const auto& boid : nearbyBoids)
	{
		float distance = Vec3::Distance(boid->position, myBoid->position); //Distance between the two boids
		if (distance < separationRadius) 
		{
			Vec3 direction = myBoid->position - boid->position; // Direction from the other boid to this boid
			direction.Normalize(direction); //Normalize the vector
			direction /= distance; //Weight the vector by the distance
			separationResult += direction; //Add the vector to the result
			separationCount++; //Increase the count
		}
	}

	if (separationCount > 0) {
		separationResult /= static_cast<float>(separationCount); //Average out the vector
	}

	// Limit the max speed (TODO)
	if (separationResult.Length() > 0.0f) { 
	  separationResult.Normalize(separationResult);
	  separationResult = separationResult * myBoid->maxSpeed; //Desired velocity
	  separationResult = separationResult - myBoid->velocity; //Steering force
	  separationResult = LimitVector(separationResult, myBoid->maxForce); //Limit the steering force
	}

	return separationResult * separationWeight; //Return the weighted steering force
}

Vec3 FlockingInfo::ComputeAlignment(const Boid* myBoid, const std::vector<Boid*>& nearbyBoids, float alignmentWeight)
{
	Vec3 alignmentResult{ 0.0f, 0.0f, 0.0f };
	int alignmentCount = 0;
	for (const auto& boid : nearbyBoids)
	{
		alignmentResult += boid->velocity; //Add the velocity of the other boid
		alignmentCount++; //Increase the count
	}

	if (alignmentCount > 0) {
		alignmentResult /= static_cast<float>(alignmentCount); //Average out the vector
		alignmentResult.Normalize(alignmentResult); //Normalize the vector
		alignmentResult = alignmentResult * myBoid->maxSpeed; //Desired velocity
		alignmentResult = alignmentResult - myBoid->velocity; //Steering force
		alignmentResult = LimitVector(alignmentResult, myBoid->maxForce); //Limit the steering force
		return alignmentResult * alignmentWeight;  //Return the weighted steering force
	}
	else
	{ 
		return Vec3(0.0f, 0.0f, 0.0f); // Return Zero Vector if there is no boids around it to align to
	}
}

Vec3 FlockingInfo::ComputeCohesion(const Boid* myBoid, const std::vector<Boid*>& nearbyBoids, float cohesionRadius, float cohesionWeight)
{
	Vec3 cohesionResult{ Vec3::Zero };
	int cohesionCount = 0;
	for (const auto& boid : nearbyBoids)
	{
		if (Vec3::Distance(boid->position, myBoid->position) < cohesionRadius)
		{
			cohesionResult += boid->position; // Add the position of the other boid
			cohesionCount++; // Increase the count
		}
	}

	if (cohesionCount > 0) {
		cohesionResult /= static_cast<float>(cohesionCount); // Average out the vector
		Vec3 steerVector = Seek(myBoid, cohesionResult); // Calculate the seek vector
		return steerVector * cohesionWeight; // Return the weighted steering force
	}
	else
	{
		return Vec3{ 0.0f,0.0f,0.0f }; // Return Zero Vector if there is no boids around it to go towards
	}
}

Vec3 FlockingInfo::ComputeWander(Boid* myBoid, float inWanderRadius, float inWanderDistance, float inWanderJitter, float inWanderWeight)
{
	Vec3 direction = Vec3(0.0f, 20.0f, 0.0f);
	myBoid->velocity.Normalize(direction);
	Vec3 circleCenter = myBoid->position + direction * inWanderDistance; // Computing the center of the circle

	 // Move the target point a bit along the circumference of the wander circle
	myBoid->wanderTarget = myBoid->wanderTarget + (Vec3{ RNG::range(-1.0f,1.0f),0.0f, RNG::range(-1.0f,1.0f) } *inWanderJitter);
	myBoid->wanderTarget.Normalize(myBoid->wanderTarget);
	myBoid->wanderTarget = myBoid->wanderTarget * inWanderRadius;

	Vec3 targetInLocalSpace = circleCenter + myBoid->wanderTarget;

	// Now we need to convert the target from local space to global space
	Vec3 targetInGlobalSpace = myBoid->position + targetInLocalSpace;

	// Calculate the desired velocity
	Vec3 desiredVelocity = Vec3(targetInGlobalSpace.x, 20.0f, targetInGlobalSpace.z) - myBoid->position;
	desiredVelocity.Normalize(desiredVelocity);
	desiredVelocity *= myBoid->maxSpeed;

	// Calculate the steering force
	Vec3 steeringForce = desiredVelocity - myBoid->velocity;
	steeringForce = LimitVector(steeringForce, myBoid->maxForce);

	return steeringForce * inWanderWeight;
}




