#pragma once
#include "BehaviorNode.h"

class L_AppearAboveGround : public BaseNode<L_AppearAboveGround>
{
public:
    L_AppearAboveGround();

protected:
    Vec3 DigDepth = Vec3(0.0f, 15.0f, 0.0f);
    Vec3 SurfaceDestination = Vec3(0.0f, 0.0f, 0.0f);
    bool LocationPreSet = false;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};