#pragma once
#include "BehaviorNode.h"

class L_DigUnderground : public BaseNode<L_DigUnderground>
{
public:
    L_DigUnderground();

protected:

	bool isDigging = false;
	Vec3 DigDepth = Vec3(0.0f, -15.0f, 0.0f);
	Vec3 DigDestination = Vec3(0.0f, 0.0f, 0.0f);

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};