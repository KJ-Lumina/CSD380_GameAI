#pragma once
#include "BehaviorNode.h"

class L_RegisterLocation : public BaseNode<L_RegisterLocation>
{
public:
    L_RegisterLocation();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};