#ifndef PP_PROJECT_H
#define PP_PROJECT_H

// cs225a includes
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

// external includes
#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <model/ModelInterface.h>

// std includes
#include <string>
#include <thread>

class PP_Project {

public:

	PP_Project(std::shared_ptr<Model::ModelInterface> robot,
		        const std::string &robot_name) :
		robot(robot),
		dof(robot->dof()),
		JOINT_TORQUES_COMMANDED_KEY(kRedisKeyPrefix + robot_name + "::actuators::fgc"),
		EE_POSITION_KEY            (kRedisKeyPrefix + robot_name + "::tasks::ee_pos"),
		EE_POSITION_DESIRED_KEY    (kRedisKeyPrefix + robot_name + "::tasks::ee_pos_des"),
		JOINT_ANGLES_KEY           (kRedisKeyPrefix + robot_name + "::sensors::q"),
		JOINT_VELOCITIES_KEY       (kRedisKeyPrefix + robot_name + "::sensors::dq"),
		TIMESTAMP_KEY              (kRedisKeyPrefix + robot_name + "::timestamp"),
		KP_POSITION_KEY            (kRedisKeyPrefix + robot_name + "::tasks::kp_pos"),
		KV_POSITION_KEY            (kRedisKeyPrefix + robot_name + "::tasks::kv_pos"),
		KP_ORIENTATION_KEY         (kRedisKeyPrefix + robot_name + "::tasks::kp_ori"),
		KV_ORIENTATION_KEY         (kRedisKeyPrefix + robot_name + "::tasks::kv_ori"),
		KP_JOINT_KEY               (kRedisKeyPrefix + robot_name + "::tasks::kp_joint"),
		KV_JOINT_KEY               (kRedisKeyPrefix + robot_name + "::tasks::kv_joint"),
		EE_ORI_KEY 				   (kRedisKeyPrefix + robot_name + "::tasks::ee_rot_mat_"),
		EE_ORI_DESIRED_KEY         (kRedisKeyPrefix + robot_name + "::tasks::ee_rot_mat_des_"),
		command_torques_(dof),
		Jv_(3, dof),
		N_(dof, dof),
		Lambda_x_(3, 3),
		g_(dof),
		q_des_(dof),
		dq_des_(dof),
		J0_(6,dof),
		L0(6,6),
		Jbar(dof, 6),
		Nbar(dof, dof),
		controller_state_(REDIS_SYNCHRONIZATION)
	{
		command_torques_.setZero();

		// Home configuration for Kuka iiwa
		//q_des_ << 90, -30, 0, 60, 0, -90, -60;
		q_des_ << 270, -30, 60, 60, 0, -90, -60;
		q_des_ *= M_PI / 180.0;
		dq_des_.setZero();

		// Desired end effector position
		x_des_ <<0.3, -0.4, 0.7;
		dx_des_.setZero();

	}

	/***** Public functions *****/

	void initialize();
	void runLoop();

protected:

	/***** Enums *****/

	// State enum for controller state machine inside runloop()
	enum ControllerState {
		REDIS_SYNCHRONIZATION,
		JOINT_SPACE_INITIALIZATION,
		OP_SPACE_POSITION_CONTROL
	};

	// Return values from computeControlTorques() methods
	enum ControllerStatus {
		RUNNING,  // Not yet converged to goal position
		FINISHED  // Converged to goal position
	};

	/***** Constants *****/

	const int dof;  // Initialized with robot model
	const double kToleranceInitQ  = 3.1;  // Joint space initialization tolerance
	const double kToleranceInitDq = 3.1;  // Joint space initialization tolerance
	const double kMaxVelocity = 0.5;  // Maximum end effector velocity

	const int kControlFreq = 1000;         // 1 kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop

	const HiredisServerInfo kRedisServerInfo = {
		"127.0.0.1",  // hostname
		6379,         // port
		{ 1, 500000 } // timeout = 1.5 seconds
	};

	// Redis keys:
	const std::string kRedisKeyPrefix = "cs225a::robot::";
	// - write:
	const std::string JOINT_TORQUES_COMMANDED_KEY;
	const std::string EE_POSITION_KEY;
	const std::string EE_POSITION_DESIRED_KEY;
	// - read:
	const std::string JOINT_ANGLES_KEY;
	const std::string JOINT_VELOCITIES_KEY;
	const std::string TIMESTAMP_KEY;
	const std::string KP_POSITION_KEY;
	const std::string KV_POSITION_KEY;
	const std::string KP_ORIENTATION_KEY;
	const std::string KV_ORIENTATION_KEY;
	const std::string KP_JOINT_KEY;
	const std::string KV_JOINT_KEY;
	const std::string KP_JOINT_INIT_KEY;
	const std::string KV_JOINT_INIT_KEY;

	const std::string EE_ORI_KEY;
	const std::string EE_ORI_DESIRED_KEY;

	/***** Member functions *****/

	void readRedisValues();
	void updateModel();
	void writeRedisValues();
	ControllerStatus computeJointSpaceControlTorques();
	ControllerStatus computeOperationalSpaceControlTorques();

	/***** Member variables *****/

	// Robot
	const std::shared_ptr<Model::ModelInterface> robot;

	// Redis
	RedisClient redis_client_;
	std::string redis_buf_;

	// Timer
	LoopTimer timer_;
	double t_curr_;
	uint64_t controller_counter_ = 0;

	// State machine
	ControllerState controller_state_;

	// Controller variables
	Eigen::VectorXd command_torques_;
	Eigen::MatrixXd Jv_;
	Eigen::MatrixXd N_;
	Eigen::MatrixXd Lambda_x_;
	Eigen::VectorXd g_;
	Eigen::Vector3d x_, dx_;
	Eigen::VectorXd q_des_, dq_des_;
	Eigen::Vector3d x_des_, dx_des_;
	
	Eigen::Matrix3d x_rot_mat_;
	Eigen::Matrix3d x_rot_mat_des_;
	Eigen::Vector3d x_w_;

	Eigen::MatrixXd J0_;
	Eigen::MatrixXd Jbar;	
	Eigen::MatrixXd Nbar;
	Eigen::MatrixXd L0;
	Eigen::MatrixXd In;

	// Default gains (used only when keys are nonexistent in Redis)
	double kp_pos_ = 150;
	double kv_pos_ = 40;
	double kp_ori_ = 150;
	double kv_ori_ = 40;
	double kp_joint_ = 40;
	double kv_joint_ = 10;
};

#endif //DEMO_PROJECT_H
