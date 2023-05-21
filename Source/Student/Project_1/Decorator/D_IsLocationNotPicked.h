#pragma once
#include "BehaviorNode.h"

class D_IsLocationNotPicked : public BaseNode<D_IsLocationNotPicked>
{
public:
    D_IsLocationNotPicked();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};