#pragma once
#include "BehaviorNode.h"

class L_WaitForTwoSecond : public BaseNode<L_WaitForTwoSecond>
{
public:
    L_WaitForTwoSecond();

protected:
	float timer = 2.0f;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
}; 
