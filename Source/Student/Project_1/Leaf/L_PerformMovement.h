#pragma once
#include "BehaviorNode.h"

class L_PerformMovement : public BaseNode<L_PerformMovement>
{
public:
    L_PerformMovement();

protected:

	bool destinationSettted = false;
	Vec3 forwardDestination = Vec3(0.0f, 0.0f, 0.0f);

    float targetAngle = 1.0f;
    float turnSpeed = 45.0f * (PI / 180.0f); // 5 degrees per second
    float turnTimer = 0.0f;
    bool timerSet = false;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
}; 
