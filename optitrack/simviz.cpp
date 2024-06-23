/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot and toro playing volleyball 
 * 
 */

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

// mutex and globals
VectorXd toro_ui_torques;
bool DELAY = true;

mutex mutex_torques, mutex_update;
int num_robots = 10;

std::string getUrdfFile(int num_robots) {
    switch (num_robots) {
        case 3:
            return "./resources/world/world_basic_3.urdf";
        case 10:
            return "./resources/world/world_basic_10.urdf";
        default:
            return "./resources/world/world_basic_1.urdf";
    }
}

static const string camera_name = "camera_fixed";


// simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main() {	
	static const string toro_file = "./resources/model/toro.urdf";

	// determine the URDF file based on the number of robots
    std::string world_file = getUrdfFile(num_robots);
    std::cout << "Loading URDF world model file: " << world_file << endl;


	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
	//graphics->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	
	// set background image
	chai3d::cBackground* background = new chai3d::cBackground();
	std::string imagePath = "../../optitrack/assets/space.jpg";
	//std::string imagePath = "/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/optitrack/assets/sea.jpg";  // Use absolute path for testing

	bool fileload = background->loadFromFile(imagePath);
	if (!fileload) {
    	std::cout << "Image File loading failed: " << imagePath << std::endl;
    	return -1;
	}

	// Add the background to the camera's back layer
	graphics->getCamera(camera_name)->m_backLayer->addChild(background);

	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 2000);  // set the near and far clipping planes 

	// add UI force interaction for each toro
    for (int i = 0; i < num_robots; ++i) {
        graphics->addUIForceInteraction("toro" + std::to_string(i));
    }

    // load robots
    vector<std::shared_ptr<Sai2Model::Sai2Model>> toros;
    for (int i = 0; i < num_robots; ++i) {
        auto toro = std::make_shared<Sai2Model::Sai2Model>(toro_file, false);
        toro->updateModel();
        toros.push_back(toro);
    }

    toro_ui_torques = VectorXd::Zero(toros[0]->dof());

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);

	for (int i = 0; i < num_robots; ++i) {
        std::string toro_name = "toro" + std::to_string(i);
        sim->setJointPositions(toro_name, toros[i]->q());
        sim->setJointVelocities(toro_name, toros[i]->dq());
    }

    // set simulation properties
    sim->setCollisionRestitution(1.0);
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

    // init redis client values 
    for (int i = 0; i < num_robots; ++i) {
        redis_client.setEigen("sai2::sim::toro" + std::to_string(i) + "::sensors::q", toros[i]->q());
        redis_client.setEigen("sai2::sim::toro" + std::to_string(i) + "::sensors::dq", toros[i]->dq());
        redis_client.setEigen("sai2::sim::toro" + std::to_string(i) + "::actuators::fgc", 0 * toros[i]->q());
    }

	/*------- Set up visualization -------*/
	// init redis client values 
	//redis_client.setEigen(TORO_JOINT_ANGLES_KEY, toro->q()); 
	//redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, toro->dq()); 
	//redis_client.setEigen(TORO_JOINT_TORQUES_COMMANDED_KEY, 0 * toro->q());

	redis_client.setBool(GLOBAL_CAM, true);

	std::string link_name = "neck_link2"; // head link
    Eigen::Affine3d transform = toros[0]->transformInWorld(link_name); // p_base = T * p_link
    MatrixXd rot = transform.rotation();
    VectorXd pos = transform.translation();
    VectorXd vert_axis = rot.col(2); // straight up from head (pos z)
    VectorXd lookat = rot.col(0); // straight ahead of head (pos x)

    VectorXd offset(3);
    offset << -2.8, 0.0, -1.1; // x = 1.6
    pos += offset;

    redis_client.setEigen(HEAD_POS, pos);
    redis_client.setEigen(HEAD_VERT_AXIS, vert_axis);
    redis_client.setEigen(HEAD_LOOK_AT, lookat + pos);

    bool conmove = true;
	thread sim_thread(simulation, sim);
	
// while window is open:
while (graphics->isWindowOpen() && fSimulationRunning) {
    for (int i = 0; i < num_robots; ++i) {
        graphics->updateRobotGraphics("toro" + std::to_string(i), redis_client.getEigen("sai2::sim::toro" + std::to_string(i) + "::sensors::q"));
    }

    // If GLOBAL_CAM is set and conmove is true, set the camera pose
    if (redis_client.getBool(GLOBAL_CAM) && conmove) {
        Eigen::Vector3d head_pos;
        head_pos << 13.0, 0.0, 4.0;
        Eigen::Vector3d head_vert_axis;
        head_vert_axis << 0.0, 0.0, 1.0;
        Eigen::Vector3d head_look_at;
        head_look_at << -1.0, 0.0, 0.0; // used to be 000

        graphics->setCameraPose(camera_name,
                                head_pos,
                                head_vert_axis,
                                head_look_at);
        
        // Set conmove to false so we only set the camera pose once
        conmove = false;
    }

    // Only update the camera pose if conmove is false
    if (conmove) {
        graphics->setCameraPose(camera_name,
                                redis_client.getEigen(HEAD_POS),
                                redis_client.getEigen(HEAD_VERT_AXIS),
                                redis_client.getEigen(HEAD_LOOK_AT));
    }
	
    graphics->renderGraphicsWorld();
    {
        lock_guard<mutex> lock(mutex_torques);
        toro_ui_torques = graphics->getUITorques("toro0");
    }
}

    // stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}


//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	// fSimulationRunning = true;

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

	// create a timer
	double sim_freq =2000;
	Sai2Common::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);

	for (int i = 0; i < num_robots; ++i) {
        sim->enableJointLimits("toro" + std::to_string(i));
    }

    while (fSimulationRunning) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();
		
        for (int i = 0; i < num_robots; ++i) {
			// Introduce delay if DELAY flag is true
            if (DELAY) {
                double start_time = i * 0.2;
                if (time < start_time) {
                    continue;
                }
            }

            VectorXd toro_control_torques = redis_client.getEigen("sai2::sim::toro" + std::to_string(i) + "::actuators::fgc");
            {
                lock_guard<mutex> lock(mutex_torques);
                sim->setJointTorques("toro" + std::to_string(i), toro_control_torques + toro_ui_torques);
            }
            sim->integrate();
            redis_client.setEigen("sai2::sim::toro" + std::to_string(i) + "::sensors::q", sim->getJointPositions("toro" + std::to_string(i)));
            redis_client.setEigen("sai2::sim::toro" + std::to_string(i) + "::sensors::dq", sim->getJointVelocities("toro" + std::to_string(i)));
        }
    }
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}
