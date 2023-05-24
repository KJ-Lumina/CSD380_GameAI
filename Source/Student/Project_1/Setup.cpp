#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"
#include "GlobalInfo.h"
#include "FlockingInfo.h"

void ProjectOne::setup()
{
    // Create your inital agents
    BehaviorAgent* carAgent = agents->create_behavior_agent("Car", BehaviorTreeTypes::Car);

    carAgent->set_color(Vec3(1.0f, 0.0f, 0.0f));
    carAgent->set_position(Vec3(7.5f, 0.0f, 7.5f));
    carAgent->set_pitch(PI / 2.0f);

    BehaviorAgent* zombieKidAgent = agents->create_behavior_agent("ZombieKid", BehaviorTreeTypes::ZombieKid);

    zombieKidAgent->set_color(Vec3(119.0f/255.0f, 120.0f/255.0f, 186.0f/255.0f)); // Purple
    zombieKidAgent->set_scaling(Vec3(1.5f, 0.8f, 1.5f));
    zombieKidAgent->set_position(Vec3(27.5f, 0.0f, 27.5f));
    zombieKidAgent->get_blackboard().set_value("TargetPosition", Vec3::Zero);

	BehaviorAgent* spinnyDollAgent = agents->create_behavior_agent("ZombieAdult", BehaviorTreeTypes::SpinnyDoll);

	spinnyDollAgent->set_color(Vec3(1.0f, 0.0f, 1.0f));
    spinnyDollAgent->set_scaling(Vec3(1.5f, 1.5f, 1.5f));
    spinnyDollAgent->set_position(Vec3(27.5f, 0.0f, 27.5f));
    spinnyDollAgent->get_blackboard().set_value("DigLocation", Vec3::Zero);

    int flockSize = 50;
    FlockingInfo::allBoids.reserve(flockSize);

    //Create Flock
    for(int i = 0; i < flockSize; ++i)
    {
		BehaviorAgent* flockAgent = agents->create_behavior_agent("Flock", BehaviorTreeTypes::Bird_Boid);

		flockAgent->set_color(Vec3(1.0f, 1.0f, 1.0f));
        flockAgent->set_scaling(Vec3(0.25f, 0.25f, 0.25f));
		flockAgent->set_position(Vec3(RNG::range(5.0f, 95.0f), 20.0f, RNG::range(5.0f, 95.0f)));
		flockAgent->set_pitch(PI / 2.0f);

        Boid boid{ static_cast<int>(flockAgent->get_id()), flockAgent->get_position(), Vec3(RNG::range(-1.0f, 1.0f), 0.0f, RNG::range(-1.0f, 1.0f)) };
		boid.agent = flockAgent;
        FlockingInfo::allBoids.emplace_back(boid);
        flockAgent->get_blackboard().set_value("MyBoid", &(FlockingInfo::allBoids[FlockingInfo::allBoids.size() - 1]));
	}

    //Map Agents
    std::vector<Vec3> WallPositions{ Vec3(15.0f,0.0f,15.0f), Vec3(15.0f,0.0f,40.0f), Vec3(40.0f, 0.0f, 40.0f), Vec3(40.0f, 0.0f,15.0f)
									,Vec3(15.0f,0.0f,60.0f), Vec3(15.0f,0.0f,85.0f), Vec3(40.0f, 0.0f, 85.0f), Vec3(40.0f, 0.0f,60.0f)
									,Vec3(60.0f,0.0f,15.0f), Vec3(60.0f,0.0f,40.0f), Vec3(85.0f, 0.0f, 40.0f), Vec3(85.0f, 0.0f,15.0f)
    								,Vec3(60.0f,0.0f,60.0f), Vec3(60.0f,0.0f,85.0f), Vec3(85.0f, 0.0f, 85.0f), Vec3(85.0f, 0.0f,60.0f) };

	std::vector<Vec3> WallScales{ Vec3(0.5f,3.0f,2.0f), Vec3(0.5f,3.0f,2.0f), Vec3(0.5f,3.0f,2.0f) ,Vec3(0.5f,3.0f,2.0f) };
    std::vector<float> WallYaw{ 0.0f, PI / 2.0f, PI, (3 * PI) / 2.0f};

    for(int i = 0; i < WallPositions.size(); ++i)
    {
		std::string name = "Wall_" + std::to_string(i);
        BehaviorAgent* agent = agents->create_behavior_agent(name.c_str(), BehaviorTreeTypes::Idle);
        agent->set_scaling(WallScales[i % WallScales.size()]);
		agent->set_position(WallPositions[i]);
		agent->set_yaw(WallYaw[i % WallYaw.size()]);
        agent->set_pitch(PI / 2.0f);
        agent->set_color(Vec3(0.0f, 0.0f, 0.0f));
    }

    {
        //Btm Left Junction
        JunctionOrientationPassasge passage1{ Vec3(-1.0f, 0.0f, 0.0f), MovementDirection::LEFT, false};
        JunctionOrientationPassasge passage2{ Vec3(0.0f, 0.0f, -1.0f), MovementDirection::RIGHT , false};
        Junction junction{ Vec3(7.5f, 0.0f, 7.5f), { passage1, passage2 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    {
        //Btm Middle Junction
        JunctionOrientationPassasge passage1{ Vec3(-1.0f, 0.0f, 0.0f), MovementDirection::LEFT , false};
        JunctionOrientationPassasge passage2{ Vec3(-1.0f, 0.0f, 0.0f), MovementDirection::RIGHT ,false};
        JunctionOrientationPassasge passage3{ Vec3(0.0f, 0.0f, -1.0f), MovementDirection::RIGHT };
        JunctionOrientationPassasge passage4{ Vec3(0.0f, 0.0f, 1.0f), MovementDirection::LEFT };
        Junction junction{ Vec3(7.5f, 0.0f, 50.0f), { passage1, passage2, passage3, passage4 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    {
        //Btm Right Junction
        JunctionOrientationPassasge passage1{ Vec3(0.0f, 0.0f, 1.0f), MovementDirection::LEFT ,false};
        JunctionOrientationPassasge passage2{ Vec3(-1.0f, 0.0f, 0.0f), MovementDirection::RIGHT,false };
        Junction junction{ Vec3(7.5f, 0.0f, 92.5f), { passage1, passage2 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    {
        //Middle Left Junction
        JunctionOrientationPassasge passage1{ Vec3(1.0f, 0.0f, 0.0f), MovementDirection::RIGHT };
        JunctionOrientationPassasge passage2{ Vec3(-1.0f, 0.0f, 0.0f), MovementDirection::LEFT };
        JunctionOrientationPassasge passage3{ Vec3(0.0f, 0.0f, -1.0f), MovementDirection::RIGHT,false };
        JunctionOrientationPassasge passage4{ Vec3(0.0f, 0.0f, -1.0f), MovementDirection::LEFT,false };
        Junction junction{ Vec3(50.0f, 0.0f, 7.5f), { passage1, passage2, passage3, passage4 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    {
        //Middle Middle Junction
        JunctionOrientationPassasge passage1{ Vec3(1.0f, 0.0f, 0.0f), MovementDirection::RIGHT };
        JunctionOrientationPassasge passage2{ Vec3(1.0f, 0.0f, 0.0f), MovementDirection::LEFT };
        JunctionOrientationPassasge passage3{ Vec3(0.0f, 0.0f, -1.0f), MovementDirection::RIGHT };
        JunctionOrientationPassasge passage4{ Vec3(0.0f, 0.0f, -1.0f), MovementDirection::LEFT };
        JunctionOrientationPassasge passage5{ Vec3(-1.0f, 0.0f, 0.0f), MovementDirection::RIGHT };
        JunctionOrientationPassasge passage6{ Vec3(-1.0f, 0.0f, 0.0f), MovementDirection::LEFT };
        JunctionOrientationPassasge passage7{ Vec3(0.0f, 0.0f, 1.0f), MovementDirection::RIGHT };
        JunctionOrientationPassasge passage8{ Vec3(0.0f, 0.0f, 1.0f), MovementDirection::LEFT };
        Junction junction{ Vec3(50.0f, 0.0f, 50.0f), { passage1, passage2, passage3, passage4, passage5, passage6, passage7, passage8 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    {
        //Middle Right Junction
        JunctionOrientationPassasge passage1{ Vec3(0.0f, 0.0f, 1.0f), MovementDirection::RIGHT,false };
        JunctionOrientationPassasge passage2{ Vec3(0.0f, 0.0f, 1.0f), MovementDirection::LEFT ,false };
        JunctionOrientationPassasge passage3{ Vec3(1.0f, 0.0f, 0.0f), MovementDirection::LEFT };
        JunctionOrientationPassasge passage4{ Vec3(-1.0f, 0.0f, 0.0f), MovementDirection::RIGHT };
        Junction junction{ Vec3(50.0f, 0.0f, 92.5f), { passage1, passage2, passage3, passage4 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    {
        //Top Left Junction
        JunctionOrientationPassasge passage1{ Vec3(1.0f, 0.0f, 0.0f), MovementDirection::RIGHT,false };
        JunctionOrientationPassasge passage2{ Vec3(0.0f, 0.0f,-1.0f), MovementDirection::LEFT ,false };
        Junction junction{ Vec3(92.5f, 0.0f, 7.5f), { passage1, passage2 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    {
        //Top Middle Junction
        JunctionOrientationPassasge passage1{ Vec3(1.0f, 0.0f, 0.0f), MovementDirection::RIGHT,false };
        JunctionOrientationPassasge passage2{ Vec3(1.0f, 0.0f, 0.0f), MovementDirection::LEFT,false };
        JunctionOrientationPassasge passage3{ Vec3(0.0f, 0.0f, 1.0f), MovementDirection::RIGHT };
        JunctionOrientationPassasge passage4{ Vec3(0.0f, 0.0f, -1.0f), MovementDirection::LEFT };
        Junction junction{ Vec3(92.5f, 0.0f, 50.0f), { passage1, passage2, passage3, passage4 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    {
        //Top Right Junction
        JunctionOrientationPassasge passage1{ Vec3(0.0f, 0.0f, 1.0f), MovementDirection::RIGHT ,false };
        JunctionOrientationPassasge passage2{ Vec3(1.0f, 0.0f, 0.0f), MovementDirection::LEFT,false };
        Junction junction{ Vec3(92.5f, 0.0f, 92.5f), { passage1, passage2 } };
        GlobalInfo::junctionPoints.push_back(junction);
    }

    for(float i = 27.5f; i <= 72.5f; i += 45.0f)
    {
		for (float j = 27.5f; j <= 72.5f; j += 45.0f)
		{
			GlobalInfo::grassPatchPosition.emplace_back(Vec3(i, 0.0f, j));
		}
    }

    // you can technically load any map you want, even create your own map file,
    // but behavior agents won't actually avoid walls or anything special, unless you code that yourself
    // that's the realm of project 2 though
    terrain->goto_map(0);

    // you can also enable the pathing layer and set grid square colors as you see fit
    // works best with map 0, the completely blank map
    terrain->pathLayer.set_enabled(true);

    std::vector<int> green{ 3,4,5,6,7,12,13,14,15,16};

    for(int i = 0; i < 20; ++i)
    {
        for(int j = 0; j < 20; ++j)
        {
            if (std::find(green.begin(), green.end(), i) != green.end() && std::find(green.begin(), green.end(), j) != green.end()) {

                terrain->pathLayer.set_value(i, j, DirectX::Colors::ForestGreen);
            }else
            {
                terrain->pathLayer.set_value(i, j, DirectX::Colors::DimGray);
            }
        }
    }



    // camera position can be modified from this default as well
    auto camera = agents->get_camera_agent();
    camera->set_position(Vec3(-62.0f, 70.0f, terrain->mapSizeInWorld * 0.5f));
    camera->set_pitch(0.610865); // 35 degrees

    audioManager->SetVolume(1.0f);
    //audioManager->PlaySoundEffect(L"Assets\\Audio\\wind.wav");
    // uncomment for example on playing music in the engine (must be .wav)
     audioManager->PlayMusic(L"Assets\\Audio\\wind.wav", true);
    // audioManager->PauseMusic(...);
    // audioManager->ResumeMusic(...);
    // audioManager->StopMusic(...);
}