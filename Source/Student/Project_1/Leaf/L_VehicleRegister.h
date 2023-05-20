#pragma once
#include "BehaviorNode.h"

class L_VehicleRegister : public BaseNode<L_VehicleRegister>
{
public:
    L_VehicleRegister();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};