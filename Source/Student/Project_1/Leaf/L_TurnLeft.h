#pragma once
#include "BehaviorNode.h"

class L_TurnLeft : public BaseNode<L_TurnLeft>
{
public:
    L_TurnLeft();

protected:

    float targetAngle = 1.0f;
    float turnSpeed = 45.0f * (PI / 180.0f); // 5 degrees per second
    float turnTimer = 0.0f;
    bool timerSet = false;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
}; 
