#include "pp_project.h"

#include <iostream>
#include <fstream>

#include <signal.h>
static volatile bool g_runloop = true;
void stop(int) { g_runloop = false; }

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

using namespace std;

/**
 * PP_Project::readRedisValues()
 * ------------------------------
 * Retrieve all read keys from Redis.
 */
void PP_Project::readRedisValues() {
	// read from Redis current sensor values
	redis_client_.REDIS_GET_EIGEN_MATRIX(JOINT_ANGLES_KEY, robot->_q);
	redis_client_.REDIS_GET_EIGEN_MATRIX(JOINT_VELOCITIES_KEY, robot->_dq);

	// Get current simulation timestamp from Redis
	redis_client_.getCommandIs(TIMESTAMP_KEY, redis_buf_);
	t_curr_ = stod(redis_buf_);

	// Read in KP and KV from Redis (can be changed on the fly in Redis)
	redis_client_.getCommandIs(KP_POSITION_KEY, redis_buf_);
	kp_pos_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KV_POSITION_KEY, redis_buf_);
	kv_pos_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KP_ORIENTATION_KEY, redis_buf_);
	kp_ori_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KV_ORIENTATION_KEY, redis_buf_);
	kv_ori_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KP_JOINT_KEY, redis_buf_);
	kp_joint_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KV_JOINT_KEY, redis_buf_);
	kv_joint_ = stoi(redis_buf_);

	// Read in desired end effector position and orientation
    redis_client_.REDIS_GET_EIGEN_MATRIX(EE_POSITION_DESIRED_KEY,x_des_);	
	redis_client_.REDIS_GET_EIGEN_MATRIX(EE_ORI_DESIRED_KEY,x_rot_mat_des_);
}

/**
 * PP_Project::writeRedisValues()
 * -------------------------------
 * Send all write keys to Redis.
 */
void PP_Project::writeRedisValues() {
	// Send end effector position and desired position
	redis_client_.REDIS_SET_EIGEN_MATRIX(EE_POSITION_KEY, x_);
	redis_client_.REDIS_SET_EIGEN_MATRIX(EE_POSITION_DESIRED_KEY, x_des_);
	
	// Send torques
	redis_client_.REDIS_SET_EIGEN_MATRIX(JOINT_TORQUES_COMMANDED_KEY, command_torques_);
}


/**
 * PP_Project::updateModel()
 * --------------------------
 * Update the robot model and all the relevant member variables.
 */
void PP_Project::updateModel() {
	// Update the model
	robot->updateModel();
	
	// Forward kinematics
	robot->position(x_, "link6", Eigen::Vector3d::Zero());
	robot->linearVelocity(dx_, "link6", Eigen::Vector3d::Zero());

	robot->rotation(x_rot_mat_, "link6"); 
	robot->angularVelocity(x_w_, "link6");
	
	// Jacobians
	Eigen::MatrixXd J0_swapped(6, robot->dof());		
	robot->J(J0_swapped, "link6", Eigen::Vector3d::Zero());
	J0_.setZero();
	J0_.topRows(3) = J0_swapped.bottomRows(3);
	J0_.bottomRows(3) = J0_swapped.topRows(3);
		
	// Dynamics
	robot->gravityVector(g_);
	L0 = (J0_ * robot->_M_inv * J0_.transpose()).inverse();
	Jbar = robot->_M_inv * J0_.transpose() * L0;
	const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof());
	Nbar = In - Jbar*J0_;
}

/**
 * PP_Project::computeJointSpaceControlTorques()
 * ----------------------------------------------
 * Controller to initialize robot to desired joint position.
 */
PP_Project::ControllerStatus PP_Project::computeJointSpaceControlTorques() {
	// Finish if the robot has converged to q_initial
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;

	if (q_err.norm() < kToleranceInitQ && dq_err.norm() < kToleranceInitDq) {
		return FINISHED;
	}

	// Compute torques
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;
	command_torques_ = robot->_M * ddq + g_;
	return RUNNING;
}

/**
 * PP_Project::computeOperationalSpaceControlTorques()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
PP_Project::ControllerStatus PP_Project::computeOperationalSpaceControlTorques() {
	



// /*	=== Position control test ===	

	// x_des_ << 0.5, 0, 0.8;
	
	// Obtain Jacobian
	Eigen::Vector3d EEdis; EEdis << 0, 0, 0;
	Eigen::MatrixXd task_jacobian; robot->Jv(task_jacobian, "link6", EEdis);

	// Obtain inertia matrix
	Eigen::MatrixXd Lambda(3, 3);
	robot->taskInertiaMatrixWithPseudoInv(Lambda, task_jacobian);

	// Obtain the current positin of end-effector
	Eigen::Vector3d curr_dx; // Current task space velocity in x, y, z
	curr_dx = task_jacobian * robot->_dq;

	// Obtain joint space gravity vector
	Eigen::VectorXd g(dof); robot->gravityVector(g);

	// Obtain end-effector position
	Eigen::Vector3d curr_x; robot->position(curr_x, "link6", EEdis);

	// Compute force at end-effector
	Eigen::Vector3d F_op;
	F_op = Lambda * (kp_pos_ * (x_des_ - curr_x) - kv_pos_*curr_dx);
		

	// Obtain nullspace matrix
	Eigen::MatrixXd N(dof, dof);
	robot->nullspaceMatrix(N, task_jacobian);

	// Compute troques and send to the robot
	Eigen::VectorXd joint_torques(dof);
	joint_torques = task_jacobian.transpose() * F_op + g - (N.transpose() * robot->_M * kv_joint_ * robot->_dq);
	command_torques_ << joint_torques;	
// */















/* 	=== Joint position control ===

	q_des_ << -1.5, 0.5, 0.5, 1, 0.5, 0.5, 0.7;
	
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;

	command_torques_ = robot->_M * (ddq) + g_;
*/

	













	/*

	// PD position control with velocity saturation
	Eigen::Vector3d x_err = x_ - x_des_;
	Eigen::Vector3d dx_err = dx_ - dx_des_;
	Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;

	//dx_des_ = -(kp_pos_ / kv_pos_) * x_err;
	//double v = kMaxVelocity / dx_des_.norm();
	//if (v > 1) v = 1;
	//Eigen::Vector3d dx_err = dx_ - v * dx_des_;
	//Eigen::Vector3d ddx = -kv_pos_ * dx_err;
	Eigen::Vector3d delta;	
	Eigen::VectorXd ee_error(6);
	Eigen::VectorXd ee_v_error(6);

	robot->orientationError(delta, x_rot_mat_des_, x_rot_mat_);
        

	ee_error << kp_pos_ * (x_des_ - x_), -1 * kp_ori_ * delta;
	ee_v_error << kv_pos_ * dx_err, kv_ori_ * x_w_;

        //cout << ee_error.norm() << endl;
        cout << (delta).norm() << endl;
	// Nullspace posture control and damping
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq ;
	Eigen::VectorXd ddq = -kp_joint_ * q_err -kv_joint_ * dq_err;

	// Control torques
	command_torques_ = J0_.transpose() * (L0 * (ee_error - ee_v_error)) + Nbar.transpose() * robot->_M * ddq + g_;
        //command_torques_ = robot->_M * ddq +  g_;
	
	*/




	return RUNNING;
}

/**
 * public PP_Project::initialize()
 * --------------------------------
 * Initialize timer and Redis client
 */
void PP_Project::initialize() {
	// Create a loop timer
	timer_.setLoopFrequency(kControlFreq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer_.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer_.initializeTimer(kInitializationPause); // 1 ms pause before starting loop

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	redis_client_.serverIs(kRedisServerInfo);
	

	// Set gains in Redis if not initialized
	if (!redis_client_.getCommandIs(KP_POSITION_KEY)) {
		redis_buf_ = to_string(kp_pos_);
		redis_client_.setCommandIs(KP_POSITION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_POSITION_KEY)) {
		redis_buf_ = to_string(kv_pos_);
		redis_client_.setCommandIs(KV_POSITION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KP_ORIENTATION_KEY)) {
		redis_buf_ = to_string(kp_ori_);
		redis_client_.setCommandIs(KP_ORIENTATION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_ORIENTATION_KEY)) {
		redis_buf_ = to_string(kv_ori_);
		redis_client_.setCommandIs(KV_ORIENTATION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KP_JOINT_KEY)) {
		redis_buf_ = to_string(kp_joint_);
		redis_client_.setCommandIs(KP_JOINT_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_JOINT_KEY)) {
		redis_buf_ = to_string(kv_joint_);
		redis_client_.setCommandIs(KV_JOINT_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(EE_ORI_DESIRED_KEY)) {
		redis_client_.REDIS_SET_EIGEN_MATRIX(EE_ORI_DESIRED_KEY, x_rot_mat_des_);
	}
}

/**
 * public PP_Project::runLoop()
 * -----------------------------
 * PP_Project state machine
 */
void PP_Project::runLoop() {
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer_.waitForNextLoop();
		++controller_counter_;
		
		// Get latest sensor values from Redis and update robot model
		readRedisValues();
		updateModel();
		 
		switch (controller_state_) {
			// Wait until valid sensor values have been published to Redis
			case REDIS_SYNCHRONIZATION:
				if (isnan(robot->_q)) continue;
				cout << "Redis synchronized. Switching to joint space controller." << endl;
				controller_state_ = JOINT_SPACE_INITIALIZATION;
				break;

			// Initialize robot to default joint configuration
			case JOINT_SPACE_INITIALIZATION:
                                cout << "Joint space intializatoin" << endl;
				if (computeJointSpaceControlTorques() == FINISHED) {
					cout << "Joint position initialized. Switching to operational space controller." << endl;
					controller_state_ = PP_Project::OP_SPACE_POSITION_CONTROL;
				};
				break;

			// Control end effector to desired position
			case OP_SPACE_POSITION_CONTROL:
				computeOperationalSpaceControlTorques();
				break;

			// Invalid state. Zero torques and exit program.
			default:
				cout << "Invalid controller state. Stopping controller." << endl;
				g_runloop = false;
				command_torques_.setZero();
				break;
		}

		// Check command torques before sending them
		if (isnan(command_torques_)) {
			cout << "NaN command torques. Sending zero torques to robot." << endl;
			command_torques_.setZero();
		}

		// Send command torques
		writeRedisValues();
	}

	// Zero out torques before quitting
	command_torques_.setZero();
	redis_client_.REDIS_SET_EIGEN_MATRIX(JOINT_TORQUES_COMMANDED_KEY, command_torques_);
}

int main(int argc, char** argv) {

	// Parse command line
	if (argc != 4) {
		cout << "Usage: pp_app <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
		exit(0);
	}
	// argument 0: executable name
	// argument 1: <path-to-world.urdf>
	string world_file(argv[1]);
	// argument 2: <path-to-robot.urdf>
	string robot_file(argv[2]);
	// argument 3: <robot-name>
	string robot_name(argv[3]);

	// Load robot
	cout << "Loading robot: " << robot_file << endl;
	auto robot = make_shared<Model::ModelInterface>(robot_file, Model::rbdl_kuka, Model::urdf, false);	
	robot->updateModel();

	// Start controller app
	cout << "Initializing app with " << robot_name << endl;
	PP_Project app(move(robot), robot_name);
	app.initialize();

	cout << "App initialized. Waiting for Redis synchronization." << endl;
	app.runLoop();

	return 0;
}

