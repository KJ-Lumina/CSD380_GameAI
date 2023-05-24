#pragma once
#include "BehaviorNode.h"

class L_UpdateFlock : public BaseNode<L_UpdateFlock>
{
public:
    L_UpdateFlock();

protected:
    float m_SeparationWeight = 2.0f;
	float m_CohesionWeight = 1.0f;
	float m_AlignmentWeight = 1.0f;
	float m_WanderWeight = 1.0f;

    float m_SeparationRadius = 10.0f;
    float m_CohesionRadius = 20.0f;
	float m_WanderRadius = 10.0f;

    float m_DetectionRadius = 40.0f;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};