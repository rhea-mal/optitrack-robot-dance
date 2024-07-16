/**
 * @file simviz.cpp
 * @brief Simulation and visualization of toro dancing 
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

bool DELAY = false;
bool fSimulationRunning = true;
std::mutex mutex_torques, mutex_update;
VectorXd toro_ui_torques;

void sighandler(int) { fSimulationRunning = false; }

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

int num_robots = 10;
static const string camera_name = "camera_fixed";
static const string toro_name = "toro0"; //first toro for controller
std::vector<string> toro_names; //map all graphics onto all

// Forward declarations
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

const std::vector<std::string> background_paths = {
    "../../optitrack/assets/space.jpg",
    "../../optitrack/assets/sea.jpg",
    "../../optitrack/assets/stanford.jpg",
    "../../optitrack/assets/trees.jpg",
    "../../optitrack/assets/wood.jpg"
};

// Function to set background
void setBackground(std::shared_ptr<Sai2Graphics::Sai2Graphics>& graphics, const std::string& imagePath) {
    chai3d::cBackground* background = new chai3d::cBackground();
    bool fileload = background->loadFromFile(imagePath);
    if (!fileload) {
        std::cerr << "Image file loading failed: " << imagePath << std::endl;
        return;
    }
    graphics->getCamera(camera_name)->m_backLayer->addChild(background);
}

int main() {	
    static const string toro_file = "./resources/model/toro.urdf";
    static const string world_file = "./resources/world/world_basic_10.urdf";
    std::cout << "Loading URDF world model file: " << world_file << endl;

    // Start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // Set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // Load graphics scene
    auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
	// set background image
	chai3d::cBackground* background = new chai3d::cBackground();
	std::string imagePath = "../../optitrack/assets/space.jpg";
	//std::string imagePath = "../.. /optitrack/assets/sea.jpg";  // Use absolute path for testing

	bool fileload = background->loadFromFile(imagePath);
	if (!fileload) {
    	std::cout << "Image File loading failed: " << imagePath << std::endl;
    	return -1;
	}



	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 2000);  // set the near and far clipping planes 
	graphics->addUIForceInteraction(toro_name);

	// load robots
	auto toro = std::make_shared<Sai2Model::Sai2Model>(toro_file, false);

	toro->updateModel();

	toro_ui_torques = VectorXd::Zero(toro->dof());

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);

	sim->setJointPositions(toro_name, toro->q());
	sim->setJointVelocities(toro_name, toro->dq());

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(1.0);
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);
		
	/*------- Set up visualization -------*/
    bool conmove = true;
    thread sim_thread(simulation, sim);

	redis_client.setEigen(TORO_JOINT_ANGLES_KEY, toro->q()); 
	redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, toro->dq()); 
	redis_client.setEigen(TORO_JOINT_TORQUES_COMMANDED_KEY, 0 * toro->q());

	// Initialize Redis keys for end effectors
	redis_client.setEigen("sai2::sim::toro::ra_end_effector_pos", Vector3d::Zero());
	redis_client.setEigen("sai2::sim::toro::la_end_effector_pos", Vector3d::Zero());
	redis_client.setInt(CLAP_KEY, 0);	
	redis_client.setBool(HANDS_ABOVE_HEAD_KEY, false);


    // While window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
		graphics->updateRobotGraphics(toro_name, redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
        
		for (int i = 0; i < num_robots; ++i) {
			std::cout << i << std::endl;
			
			// if (DELAY) {
            //     double sleep_time = i * 2;
            //     sleep(sleep_time);
            // }

			std::string name = toro_names[i];

			//graphics->updateRobotGraphics(name, redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
			//std::cout << "Updating graphics for " << name << " with joint positions: " << TORO_JOINT_ANGLES_KEY.transpose() << std::endl;
		}

		// Retrieve the positions of the right and left end effectors (hands)
		Eigen::Vector3d ra_end_effector_pos = redis_client.getEigen("sai2::sim::toro::ra_end_effector_pos");
		Eigen::Vector3d la_end_effector_pos = redis_client.getEigen("sai2::sim::toro::la_end_effector_pos");

		// Calculate the L2 norm of the difference between the end effectors
		double l2_norm = (ra_end_effector_pos - la_end_effector_pos).norm();

		// Check if the L2 norm is below the clap threshold
        //if (l2_norm < CLAP_THRESHOLD) {
		if (false) {
            int current_clap_count = redis_client.getInt(CLAP_KEY);
            redis_client.setInt(CLAP_KEY, current_clap_count + 1);

            // Change background based on clap count
            int background_index = (current_clap_count + 1) % background_paths.size();
            setBackground(graphics, background_paths[background_index]);
        }

		// Retrieve the head position
		Eigen::Vector3d head_pos = redis_client.getEigen("sai2::sim::toro::head_pos");

		// Check if both hands are above the head
		bool hands_above_head = (ra_end_effector_pos.z() > head_pos.z() + HANDS_ABOVE_HEAD_THRESHOLD &&
								la_end_effector_pos.z() > head_pos.z() + HANDS_ABOVE_HEAD_THRESHOLD);
		redis_client.setBool(HANDS_ABOVE_HEAD_KEY, hands_above_head);
		
		// Set transparency for odd-numbered robots if hands are above the head
		if (hands_above_head) {
			for (int i = 0; i < num_robots; ++i) {
				if (i % 2 == 1) { // Odd robots
					//graphics->showTransparency(true, toro_names[i], 0.5); // Set transparency level to 0.5
				}
			}
		} 
		// else {
		// 	for (int i = 0; i < num_robots; ++i) {
		// 		if (i % 2 == 1) { // Odd robots
		// 			//graphics->showTransparency(false, toro_names[i], 1.0); // Set transparency level to 1.0 (fully opaque)
		// 		}
		// 	}
		// }

		if (redis_client.getBool(GLOBAL_CAM) && conmove) {
			Eigen::Vector3d head_pos;
			head_pos << 2.0, -0.8, 6.0;
			Eigen::Vector3d head_vert_axis;
			head_vert_axis << 0.0, 0.0, 1.0;
			Eigen::Vector3d head_look_at;
			head_look_at << 0.0, 0.0, 0.0;

			// Create the rotation matrix
			Eigen::Matrix3d rotation;
			rotation.col(2) = head_vert_axis.normalized(); // z-axis
			rotation.col(0) = head_look_at.normalized(); // x-axis
			rotation.col(1) = rotation.col(2).cross(rotation.col(0)).normalized(); // y-axis

			// Create the Affine3d object
			Eigen::Affine3d camera_pose = Eigen::Affine3d::Identity();
			camera_pose.translation() = head_pos;
			camera_pose.linear() = rotation;

			// Call setCameraPose with the correct arguments
			//graphics->setCameraPose(camera_name, camera_pose);
			conmove = false;
		} else {
			// Retrieve the head position, vertical axis, and look-at direction
			Eigen::Vector3d head_pos = redis_client.getEigen(HEAD_POS);
			Eigen::Vector3d head_vert_axis = redis_client.getEigen(HEAD_VERT_AXIS);
			Eigen::Vector3d head_look_at = redis_client.getEigen(HEAD_LOOK_AT);

			// Create the rotation matrix
			Eigen::Matrix3d rotation;
			rotation.col(2) = head_vert_axis.normalized(); // z-axis
			rotation.col(0) = head_look_at.normalized(); // x-axis
			rotation.col(1) = rotation.col(2).cross(rotation.col(0)).normalized(); // y-axis

			// Create the Affine3d object
			Eigen::Affine3d camera_pose = Eigen::Affine3d::Identity();
			camera_pose.translation() = head_pos;
			camera_pose.linear() = rotation;
			//graphics->setCameraPose(camera_name, camera_pose);
			conmove = true;
		}

		
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			toro_ui_torques = graphics->getUITorques(toro_name);
		}
	}

	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}


//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    double sim_freq = 2000;
    Sai2Common::LoopTimer timer(sim_freq);

    sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);

    // Enable joint limits for each toro 
    for (const auto& name : toro_names) {
        sim->enableJointLimits(name);
    }

    while (fSimulationRunning) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

		VectorXd toro_control_torques = redis_client.getEigen(TORO_JOINT_TORQUES_COMMANDED_KEY);
		// ONLY CONTROL 1 ROBOT HERE 
        // Iterate over each toro robot to set torques
		
		{
            lock_guard<mutex> lock(mutex_torques);
            //sim->setJointTorques(name, toro_control_torques + toro_ui_torques[i]);
			
			//sim->setJointTorques(toro_name, toro_control_torques + toro_ui_torques);
			for (int i = 0; i < num_robots; ++i) {
				std::string name = "toro" + std::to_string(i);
				sim->setJointTorques(name, toro_control_torques + toro_ui_torques);
			}
        }
		sim->integrate();
        
		redis_client.setEigen(TORO_JOINT_ANGLES_KEY, sim->getJointPositions(toro_name));
        redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, sim->getJointVelocities(toro_name));

        {
            lock_guard<mutex> lock(mutex_update);
            
        }
    }

    timer.stop();
    cout << "\nSimulation loop timer stats:\n";
    timer.printInfoPostRun();
}
