#include <pch.h>
#include "L_PlaySound_Spinny.h"

void L_PlaySound_Spinny::on_enter()
{
	audioManager->PlaySoundEffect(L"Assets\\Audio\\kurukuru.wav");
	BehaviorNode::on_leaf_enter();
	on_success();
}