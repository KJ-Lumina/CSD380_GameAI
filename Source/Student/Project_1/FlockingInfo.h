#pragma once
#include "Blackboard.h"

struct Boid
{
	Boid() = default;
	~Boid() = default;
	Boid(int inAgentID, Vec3 inPosition, Vec3 inVelocity) : agentId(inAgentID), position(inPosition), velocity(inVelocity) {}

	int agentId{ -1 }; // The id of the agent this boid is reference to
	BehaviorAgent* agent{ nullptr }; // The agent this boid is reference to

	Vec3 position{ 0.0f, 0.0f, 0.0f };
	Vec3 velocity{ 0.0f, 0.0f, 0.0f };
	Vec3 acceleration{ 0.0f, 0.0f, 0.0f };

	float wanderJitter = 0.5f; // The rate at which the wander target changes
	float wanderDistance = 1.0f; // The distance the wander circle is projected in front of the agent
	Vec3 wanderTarget = Vec3(0.0f,0.0f,0.0f); // The target for the wander behavior so that we can move in the small circle

	float maxSpeed{ 24.0f };
	float maxForce{ 0.18f };
};

class FlockingInfo
{
public:
	inline static std::vector<Boid> allBoids; // The boids for flocking

	static Vec3 ComputeSeparation(const Boid* myBoid, const std::vector<Boid*>& nearbyBoids, float separationRadius, float separationWeight);
	static Vec3 ComputeAlignment(const Boid* myBoid, const std::vector<Boid*>& nearbyBoids, float alignmentWeight);
	static Vec3 ComputeCohesion(const Boid* myBoid, const std::vector<Boid*>& nearbyBoids, float cohesionRadius, float cohesionWeight);
	static Vec3 ComputeWander(Boid* myBoid, float wanderRadius, float wanderDistance, float wanderJitter, float wanderWeight);
};