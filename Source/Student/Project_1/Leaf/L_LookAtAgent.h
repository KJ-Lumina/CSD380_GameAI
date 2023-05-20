#pragma once
#include "BehaviorNode.h"

class L_LookAtAgent : public BaseNode<L_LookAtAgent>
{
public:
    L_LookAtAgent();

protected:
	Vec3 targetAgent;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
}; 
