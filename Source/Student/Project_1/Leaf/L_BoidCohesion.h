#pragma once
#include "BehaviorNode.h"

class L_BoidCohesion : public BaseNode<L_BoidCohesion>
{
public:
    L_BoidCohesion();

protected:
	float m_CohesionRadius = 10.0f;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};