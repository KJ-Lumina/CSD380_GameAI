#pragma once
#include "BehaviorNode.h"

class D_PossibleToTurnRight : public BaseNode<D_PossibleToTurnRight>
{
public:
    D_PossibleToTurnRight();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};