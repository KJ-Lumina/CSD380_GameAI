#pragma once
#include "BehaviorNode.h"

class D_IsVehicleRegistered : public BaseNode<D_IsVehicleRegistered>
{
public:
    D_IsVehicleRegistered();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};