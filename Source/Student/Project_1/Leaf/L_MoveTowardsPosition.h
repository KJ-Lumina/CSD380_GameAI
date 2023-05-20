#pragma once
#include "BehaviorNode.h"

class L_MoveTowardsPosition : public BaseNode<L_MoveTowardsPosition>
{
public:
    L_MoveTowardsPosition();

    Vec3 targetPosition;
protected:
    virtual void on_enter() override;
    virtual void on_update(float dt) override;
}; 