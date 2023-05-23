#pragma once

// Include all node headers in this file

// Example Control Flow Nodes
#include "ControlFlow/C_ParallelSequencer.h"
#include "ControlFlow/C_RandomSelector.h"
#include "ControlFlow/C_Selector.h"
#include "ControlFlow/C_Sequencer.h"

// Student Control Flow Nodes


// Example Decorator Nodes
#include "Decorator/D_Delay.h"
#include "Decorator/D_InvertedRepeater.h"
#include "Decorator/D_RepeatFourTimes.h"

// Student Decorator Nodes
#include "Decorator/D_IsAtJunction.h"
#include "Decorator/D_IsVehicleRegistered.h"
#include "Decorator/D_IsMovementRegistered.h"
#include "Decorator/D_IsLocationNotPicked.h"
#include "Decorator/D_IsRandomLocationNotPicked.h"
#include "Decorator/D_IsThereBirdNeighbours.h"

// Example Leaf Nodes
#include "Leaf/L_CheckMouseClick.h"
#include "Leaf/L_Idle.h"
#include "Leaf/L_MoveToFurthestAgent.h"
#include "Leaf/L_MoveToMouseClick.h"
#include "Leaf/L_MoveToRandomPosition.h"

// Student Leaf Nodes
#include "Leaf/L_MoveTowardsPosition.h"
#include "Leaf/L_LookAtAgent.h"
#include "Leaf/L_PerformMovement.h"
#include "Leaf/L_WaitForTwoSecond.h"
#include "Leaf/L_WaitForOneSecond.h"
#include "Leaf/L_IsRoadClear.h"
#include "Leaf/L_VehicleRegister.h"
#include "Leaf/L_RegisterLeftTurn.h"
#include "Leaf/L_RegisterRightTurn.h"
#include "Leaf/L_SelectMovement.h"
#include "Leaf/L_RegisterStraight.h"
#include "Leaf/L_RegisterLocation.h"
#include "Leaf/L_UnregisterLocation.h"
#include "Leaf/L_IsThereVehicleInfront.h"
#include "Leaf/L_LookTowardCamera.h"
#include "Leaf/L_RegisterRandomPatchLocation.h"
#include "Leaf/L_UnregisterRandomPatchLocation.h"
#include "Leaf/L_AppearAboveGround.h"
#include "Leaf/L_DigUnderground.h"
#include "Leaf/L_Spin.h"
#include "Leaf/L_PlaySound_Zombie.h"
#include "Leaf/L_PlaySound_Spinny.h"
#include "Leaf/L_PlaySound_Gawk.h"
#include "Leaf/L_UpdateFlock.h"
