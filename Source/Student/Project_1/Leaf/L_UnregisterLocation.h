#pragma once
#include "BehaviorNode.h"

class L_UnregisterLocation : public BaseNode<L_UnregisterLocation>
{
public:
    L_UnregisterLocation();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};