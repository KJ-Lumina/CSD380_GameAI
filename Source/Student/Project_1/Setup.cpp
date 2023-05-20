#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"
#include "GlobalInfo.h"

void ProjectOne::setup()
{
    // Create your inital agents
    BehaviorAgent* _agent = agents->create_behavior_agent("ExampleAgent", BehaviorTreeTypes::Car);

	_agent->set_position(Vec3(7.5f, 0.0f, 7.5f));
	_agent->set_pitch(PI / 2.0f);

	/*BehaviorAgent* _agent = agents->create_behavior_agent("ExampleAgent2", BehaviorTreeTypes::Idle);
    _agent->set_position(Vec3(7.5f, 0.0f, 7.5f));
    _agent->set_pitch(PI / 2.0f);
	_agent->set_yaw(PI / 2.0f);
	std::cout << "Agent Up Vector: " << _agent->get_up_vector().x << ", " << _agent->get_up_vector().y << ", " << _agent->get_up_vector().z << std::endl;*/

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

    audioManager->SetVolume(0.5f);
    audioManager->PlaySoundEffect(L"Assets\\Audio\\retro.wav");
    // uncomment for example on playing music in the engine (must be .wav)
    // audioManager->PlayMusic(L"Assets\\Audio\\motivate.wav");
    // audioManager->PauseMusic(...);
    // audioManager->ResumeMusic(...);
    // audioManager->StopMusic(...);
}