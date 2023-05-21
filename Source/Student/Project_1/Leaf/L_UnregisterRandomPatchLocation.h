#pragma once
#include "BehaviorNode.h"

class L_UnregisterRandomPatchLocation : public BaseNode<L_UnregisterRandomPatchLocation>
{
public:
    L_UnregisterRandomPatchLocation();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};