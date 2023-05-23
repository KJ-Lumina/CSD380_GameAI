#pragma once
#include "BehaviorNode.h"

class D_IsThereBirdNeighbours : public BaseNode<D_IsThereBirdNeighbours>
{
public:
    D_IsThereBirdNeighbours();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};