#include <pch.h>
#include "L_PlaySound_Gawk.h"

void L_PlaySound_Gawk::on_enter()
{
	int maxCount = 3000 + agent->get_blackboard().get_value<int>("BirdNeighbours");

	int random = RNG::range(0, maxCount);

	if (random == 0) {
		audioManager->SetVolume(0.3f);
		audioManager->PlaySoundEffect(L"Assets\\Audio\\gawkgawk.wav");
		audioManager->SetVolume(0.5f);
	}

	BehaviorNode::on_leaf_enter();
	on_success();
}