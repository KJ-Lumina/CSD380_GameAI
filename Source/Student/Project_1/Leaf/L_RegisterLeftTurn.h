#pragma once
#include "BehaviorNode.h"

class L_RegisterLeftTurn : public BaseNode<L_RegisterLeftTurn>
{
public:
    L_RegisterLeftTurn();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};