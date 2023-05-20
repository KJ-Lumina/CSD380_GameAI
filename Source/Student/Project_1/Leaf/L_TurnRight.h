#pragma once
#include "BehaviorNode.h"

class L_TurnRight : public BaseNode<L_TurnRight>
{
public:
    L_TurnRight();

protected:
    float targetAngle = 1.0f;
	float turnSpeed = 20.0f * (PI / 180.0f); // 5 degrees per second
    float turnTimer = 0.0f;
	bool timerSet = false;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
}; 
