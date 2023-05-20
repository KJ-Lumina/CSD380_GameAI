#pragma once
#include "BehaviorNode.h"

class L_SelectMovement : public BaseNode<L_SelectMovement>
{
public:
    L_SelectMovement();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};