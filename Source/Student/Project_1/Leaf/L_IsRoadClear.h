#pragma once
#include "BehaviorNode.h"

class L_IsRoadClear : public BaseNode<L_IsRoadClear>
{
public:
    L_IsRoadClear();

protected:
    float visualCheckDistance = 5.0f;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
}; 
