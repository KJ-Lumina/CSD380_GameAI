#pragma once
#include "BehaviorNode.h"

class L_LookTowardCamera : public BaseNode<L_LookTowardCamera>
{
public:
    L_LookTowardCamera();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};