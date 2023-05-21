#pragma once
#include "BehaviorNode.h"

class L_PlaySound_Zombie : public BaseNode<L_PlaySound_Zombie>
{
protected:
    virtual void on_enter() override;
};