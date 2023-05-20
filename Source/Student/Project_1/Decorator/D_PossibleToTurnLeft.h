#pragma once
#include "BehaviorNode.h"

class D_PossibleToTurnLeft : public BaseNode<D_PossibleToTurnLeft>
{
public:
    D_PossibleToTurnLeft();

protected:;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};