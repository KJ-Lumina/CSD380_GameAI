#pragma once
#include "BehaviorNode.h"

class D_IsRandomLocationNotPicked : public BaseNode<D_IsRandomLocationNotPicked>
{
public:
    D_IsRandomLocationNotPicked();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};