#pragma once
#include "BehaviorNode.h"

class L_Spin : public BaseNode<L_Spin>
{
public:
    L_Spin();

protected:

    float spinTimer = 1.0f;
    float spinSpeed = 8 * PI;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};