#pragma once
#include "BehaviorNode.h"

class L_GetAllNearbyBoid : public BaseNode<L_GetAllNearbyBoid>
{
public:
    L_GetAllNearbyBoid();

protected:

    float m_DetectionRadius = 20.0f;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};