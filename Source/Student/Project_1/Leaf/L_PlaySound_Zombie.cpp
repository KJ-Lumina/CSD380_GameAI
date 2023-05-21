#include <pch.h>
#include "L_PlaySound_Zombie.h"

void L_PlaySound_Zombie::on_enter()
{
	audioManager->PlaySoundEffect(L"Assets\\Audio\\wakaranai.wav");
	BehaviorNode::on_leaf_enter();
	on_success();
}