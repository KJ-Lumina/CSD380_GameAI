#pragma once
#include "BehaviorNode.h"

class L_BoidAlignment : public BaseNode<L_BoidAlignment>
{
public:
    L_BoidAlignment();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};