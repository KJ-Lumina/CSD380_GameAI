#pragma once
#include "BehaviorNode.h"

class L_RegisterRightTurn : public BaseNode<L_RegisterRightTurn>
{
public:
    L_RegisterRightTurn();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};