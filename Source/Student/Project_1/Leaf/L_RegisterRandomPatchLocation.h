#pragma once
#include "BehaviorNode.h"

class L_RegisterRandomPatchLocation : public BaseNode<L_RegisterRandomPatchLocation>
{
public:
    L_RegisterRandomPatchLocation();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};