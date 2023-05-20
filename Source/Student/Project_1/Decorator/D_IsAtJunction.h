#pragma once
#include "BehaviorNode.h"

class D_IsAtJunction : public BaseNode<D_IsAtJunction>
{
public:
    D_IsAtJunction();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};