/**
 * @file simviz.cpp
 * @brief Simulation and visualization of dancing toro 
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
#include <chrono>

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}
double DELAY = 2000; // simulation frequency in Hz
std::shared_ptr<Sai2Model::Sai2Model> toro;
std::shared_ptr<Sai2Graphics::Sai2Graphics> graphics;


#include "redis_keys.h"

using namespace Eigen;
using namespace std;


// mutex and globals
VectorXd toro_ui_torques;

mutex mutex_torques, mutex_update;

// specify urdf and robots 
static const string toro_name = "toro5";
static const string camera_name = "camera_fixed";

const std::vector<std::string> background_paths = {
    "../../optitrack/assets/space.jpg",
    "../../optitrack/assets/sea.jpg",
    "../../optitrack/assets/stanford.jpg",
    "../../optitrack/assets/trees.jpg",
    "../../optitrack/assets/wood.jpg"
};

void setBackgroundImage(std::shared_ptr<Sai2Graphics::Sai2Graphics>& graphics, const std::string& imagePath) {
    chai3d::cBackground* background = new chai3d::cBackground();
    bool fileload = background->loadFromFile(imagePath);
    if (!fileload) {
        std::cerr << "Image file loading failed: " << imagePath << std::endl;
        return;
    }
    graphics->getCamera(camera_name)->m_backLayer->addChild(background);
}

chai3d::cColorf lagrangianToColor(double lagrangian, double min_lagrangian, double max_lagrangian) {
    double normalized = (lagrangian - min_lagrangian) / (max_lagrangian - min_lagrangian);
    normalized = std::max(0.0, std::min(1.0, normalized)); // Clamp to [0, 1]

    // Blue to Red gradient
    double red = normalized;
    double blue = 1.0 - normalized;
    double green = 0.0;

    return chai3d::cColorf(red, green, blue);
}


// simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);



int main() {
	static const string toro_file = "./resources/model/toro.urdf";
    static const string world_file = "./resources/world/world_basic_10.urdf";
    std::cout << "Loading URDF world model file: " << world_file << endl;

    // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load graphics scene
    graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
    //setBackgroundImage(graphics, "../../optitrack/assets/space.jpg"); // Set background to space
    
    graphics->getCamera(camera_name)->setClippingPlanes(0.1, 2000);  // set the near and far clipping planes 

    // load robots
    toro = std::make_shared<Sai2Model::Sai2Model>(toro_file, false);
    toro->updateModel();
    toro_ui_torques = VectorXd::Zero(toro->dof());

	//graphics->addUIForceInteraction(toro_name);

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
	sim->setJointPositions(toro_name, toro->q());
	sim->setJointVelocities(toro_name, toro->dq());


    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(1.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 

	redis_client.setEigen(TORO_JOINT_ANGLES_KEY, toro->q()); 
	redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, toro->dq()); 
	redis_client.setEigen(TORO_JOINT_TORQUES_COMMANDED_KEY, 0 * toro->q());


	string link_name = "neck_link2"; // head link
	Eigen::Affine3d transform = toro->transformInWorld(link_name); // p_base = T * p_link
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

	// start simulation thread
	thread sim_thread(simulation, sim);
	
	int robot_index = 0; // index to track which robot to update next
    int total_robots = 10; // total number of robots to update

    Sai2Common::LoopTimer timer(DELAY);
    timer.reinitializeTimer(1e9);

	bool changed_recently = false;
	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
		timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();


		// Get the Lagrangian value from Redis
		double lagrangian = stod(redis_client.get(LAGRANGIAN));

		//double normalized_lagrangian = (lagrangian + 50.0) / 100.0;
		// double red = fmin(fmax(0.0, lagrangian), 1.0);
		// double green = 0.3;
		// double blue = fmin(fmax(0.0, 1.0 - lagrangian), 1.0);

        // Set the background color based on the Lagrangian value
        //chai3d::cColorf backgroundColor(red, green, blue);
		//graphics->setBackgroundColor(red, green, blue);
		chai3d::cColorf backgroundColor = lagrangianToColor(lagrangian, -50.0, 200.0);
		double red = backgroundColor.getR();
    	double green = backgroundColor.getG();
   		double blue = backgroundColor.getB();
		graphics->setBackgroundColor(red, green, blue);
        
        // Update primary robot graphics
        graphics->updateRobotGraphics(toro_name, redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
        
        // Update one robot graphics per iteration
        std::string name = "toro" + std::to_string(robot_index);
        graphics->updateRobotGraphics(name, redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
        robot_index = (robot_index + 1) % total_robots;


		if (robot_index == 8) {
			changed_recently = false; // better logic for this
		}
        
		// Retrieve the positions of the right and left end effectors (hands)
		Eigen::Vector3d ra_end_effector_pos = redis_client.getEigen("sai2::optitrack::rigid_body_pos::6");
		Eigen::Vector3d la_end_effector_pos = redis_client.getEigen("sai2::optitrack::rigid_body_pos::3");

		// Calculate the L2 norm of the difference between the end effectors
		double l2_norm = (ra_end_effector_pos - la_end_effector_pos).norm();
		
		double last_background_change_time = 0.0; // Initialize the last background change time

		if (l2_norm < CLAP_THRESHOLD) {
			if (!changed_recently) {
				int current_clap_count = redis_client.getInt(CLAP_KEY);
				redis_client.setInt(CLAP_KEY, current_clap_count + 1);

				int background_index = (current_clap_count + 1) % background_paths.size();
				//UNCOMMENT THIS
				//setBackgroundImage(graphics, background_paths[background_index]);

				// Update the last background change time
				last_background_change_time = time;
				changed_recently = true;

				//std::cout << "Background changed to: " << background_paths[background_index] << std::endl;
			}
		}

		// Retrieve the head position
		Eigen::Vector3d head_pos = redis_client.getEigen("sai2::sim::toro::head_pos");

		// Check if both hands are above the head
		bool hands_above_head = (head_pos.z() - ra_end_effector_pos.z() < HANDS_ABOVE_HEAD_THRESHOLD ||
								head_pos.z() - la_end_effector_pos.z() < HANDS_ABOVE_HEAD_THRESHOLD);
		redis_client.setBool(HANDS_ABOVE_HEAD_KEY, hands_above_head);

		double hands_above = head_pos.z() - ra_end_effector_pos.z();
		
		//std::cout << head_pos.z() - ra_end_effector_pos.z() << std::endl;
		// Set transparency for odd-numbered robots if hands are above the head
		if (hands_above < 0.1) {
			graphics->showTransparency(true, "toro0", 0.0);
			graphics->showTransparency(true, "toro1", 0.0);
			graphics->showTransparency(true, "toro2", 0.0); // Fully transparent
			graphics->showTransparency(true, "toro3", 0.0); // Fully transparent
			graphics->showTransparency(true, "toro4", 0.0);
			graphics->showTransparency(true, "toro5", 1.0);
			graphics->showTransparency(true, "toro6", 0.0); // Fully transparent
			graphics->showTransparency(true, "toro7", 0.0); // Fully transparent
			graphics->showTransparency(true, "toro8", 0.0); // Fully transparent
			graphics->showTransparency(true, "toro9", 0.0); // Fully transparent
		} 
		else if (0.3 > hands_above > 0.1) {
			graphics->showTransparency(true, "toro0", 0.0);
			graphics->showTransparency(true, "toro1", 0.0);
			graphics->showTransparency(true, "toro2", 1.0); // Fully transparent
			graphics->showTransparency(true, "toro3", 1.0); // Fully transparent
			graphics->showTransparency(true, "toro4", 1.0);
			graphics->showTransparency(true, "toro4", 1.0);
			graphics->showTransparency(true, "toro5", 1.0);
			graphics->showTransparency(true, "toro6", 1.0); // Fully transparent
			graphics->showTransparency(true, "toro7", 1.0); // Fully transparent
			graphics->showTransparency(true, "toro8", 0.0); // Fully transparent
			graphics->showTransparency(true, "toro9", 0.0); // Fully transparent
		} 
		else if (0.6 > hands_above > 0.3) {
			graphics->showTransparency(true, "toro0", 1.0);
			graphics->showTransparency(true, "toro1", 0.0);
			graphics->showTransparency(true, "toro2", 1.0); // Fully transparent
			graphics->showTransparency(true, "toro3", 0.0); // Fully transparent
			graphics->showTransparency(true, "toro4", 1.0);
			graphics->showTransparency(true, "toro5", 0.0);
			graphics->showTransparency(true, "toro6", 1.0); // Fully transparent
			graphics->showTransparency(true, "toro7", 0.0); // Fully transparent
			graphics->showTransparency(true, "toro8", 1.0); // Fully transparent
			graphics->showTransparency(true, "toro9", 0.0); 
		}
		else {
			graphics->showTransparency(true, "toro0", 1.0);
			graphics->showTransparency(true, "toro1", 1.0);
			graphics->showTransparency(true, "toro2", 1.0); 
			graphics->showTransparency(true, "toro3", 1.0); 
			graphics->showTransparency(true, "toro4", 1.0);
			graphics->showTransparency(true, "toro5", 1.0);
			graphics->showTransparency(true, "toro6", 1.0); 
			graphics->showTransparency(true, "toro7", 1.0);
			graphics->showTransparency(true, "toro8", 1.0); 
			graphics->showTransparency(true, "toro9", 1.0); 
		}
    
        {
			lock_guard<mutex> lock(mutex_update);
			
		}

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
			graphics->setCameraPose(camera_name, camera_pose);
			conmove = false;
		} else if (redis_client.getBool(GLOBAL_CAM) && !conmove) {

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

			// Call setCameraPose with the correct arguments
			graphics->setCameraPose(camera_name, camera_pose);
			conmove = true;
		}

		
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			
			toro_ui_torques = graphics->getUITorques(toro_name);
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
    double sim_freq = 2000;
    Sai2Common::LoopTimer timer(sim_freq);

    sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);

    sim->enableJointLimits(toro_name);

    while (fSimulationRunning) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        VectorXd toro_control_torques = redis_client.getEigen(TORO_JOINT_TORQUES_COMMANDED_KEY);
        {
            lock_guard<mutex> lock(mutex_torques);
            sim->setJointTorques(toro_name, toro_control_torques + toro_ui_torques);
        }
        sim->integrate();

		VectorXd robot_q = sim->getJointPositions(toro_name);
    	VectorXd robot_dq = sim->getJointVelocities(toro_name);
        //robot_dq = 0.1 * VectorXd::Ones(robot_dq.size()) * sin(time);

        // Get the mass matrix
        MatrixXd robot_M = toro->M();
        VectorXd g = toro->jointGravityVector();
        double kinetic_energy = 0.5 * robot_dq.transpose() * robot_M * robot_dq;
        double potential_energy = -robot_q.transpose() * g;
        double lagrangian = kinetic_energy - potential_energy;
        
        std::cout << "Lagrangian: " << lagrangian << std::endl;
		//redis_client.setEigen(LAGRANGIAN, to_string(lagrangian));
		// After calculating the Lagrangian
		redis_client.set(LAGRANGIAN, std::to_string(lagrangian));

        redis_client.setEigen(TORO_JOINT_ANGLES_KEY, robot_q);
        redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, robot_dq);

        {
            lock_guard<mutex> lock(mutex_update);
        }
    }
    timer.stop();
    cout << "\nSimulation loop timer stats:\n";
    timer.printInfoPostRun();
}

