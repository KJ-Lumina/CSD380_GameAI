#pragma once
#include "BehaviorNode.h"

class D_IsMovementRegistered : public BaseNode<D_IsMovementRegistered>
{
public:
    D_IsMovementRegistered();

protected:

    bool checked = false;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};