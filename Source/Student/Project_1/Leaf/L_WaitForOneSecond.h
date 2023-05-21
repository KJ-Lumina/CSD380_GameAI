#pragma once
#include "BehaviorNode.h"

class L_WaitForOneSecond : public BaseNode<L_WaitForOneSecond>
{
public:
    L_WaitForOneSecond();

protected:
	float timer = 1.0f;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
}; 
