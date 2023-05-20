#pragma once
#include "BehaviorNode.h"

class L_RegisterStraight : public BaseNode<L_RegisterStraight>
{
public:
    L_RegisterStraight();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};