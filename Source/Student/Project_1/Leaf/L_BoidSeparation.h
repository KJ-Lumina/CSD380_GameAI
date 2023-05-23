#pragma once
#include "BehaviorNode.h"

class L_BoidSeparation : public BaseNode<L_BoidSeparation>
{
public:
    L_BoidSeparation();

protected:
	float m_SeparationRadius = 10.0f;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};