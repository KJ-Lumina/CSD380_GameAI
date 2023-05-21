#pragma once
#include "BehaviorNode.h"

class L_IsThereVehicleInfront : public BaseNode<L_IsThereVehicleInfront>
{
public:
    L_IsThereVehicleInfront();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};