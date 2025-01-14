// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "utils.h"
#include "kdl_parser/kdl_parser.hpp"

#include <Eigen/Geometry> // For Eigen::Quaterniond

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    Iiwa_pub_sub(int trajectory_index)
    : Node("ros2_kdl_vision_control"),
          trajectory_index_(trajectory_index)    
    {
        // Declare cmd_interface parameter (position, velocity)
        declare_parameter("cmd_interface", "position"); // defaults to "position"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(), "Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
        {
            RCLCPP_ERROR(get_logger(), "Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
            return;
        }

        // Declare the parameter "task"
        declare_parameter("task", "niente"); // Default is "niente"
        get_parameter("task", task_);
        RCLCPP_INFO(get_logger(), "Current task is: '%s'", task_.c_str());

        // Validate the task parameter
        if (!(task_ == "positioning" || task_ == "look-at-point") && cmd_interface_ == "velocity")
        {
            RCLCPP_ERROR(get_logger(), "Invalid task selected! Use 'positioning' or 'look-at-point' instead.");
            return;
        }

	// Declare control_space parameter
	declare_parameter("control_space", "niente"); // default to "niente"
	get_parameter("control_space", control_space_);
	RCLCPP_INFO(get_logger(), "Current control space is: '%s'", control_space_.c_str());

	if (!(control_space_ == "joint_space" || control_space_ == "operational_space") && cmd_interface_ == "effort")
	{
	    RCLCPP_ERROR(get_logger(), "Selected control_space is not valid! Use 'joint_space' or 'operational_space'.");
	    return;
	}
	
        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;
        aruco_pose_available_ = false;
        posizione_iniziale_raggiunta_ = false;
        
        // Retrieve robot_description param
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        // Create KDLrobot structure
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree))
        {
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        // Create joint array
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96; // TODO: read from urdf file
        q_max.data <<  2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96; // TODO: read from urdf file
        robot_->setJntLimits(q_min, q_max);

        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_cmd_.data.setZero();

        // Subscriber to jnt states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1)
        );

        // Subscribe to the ArUco marker pose
        aruco_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_detect/pose",
            500,
            std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1)
        );

        // Wait for the joint_state topic
        while (!joint_state_available_)
        {
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(this->get_node_base_interface());
        }

        // Define the initial joint positions (desired positions)
        init_joint_positions_.resize(nj);
        init_joint_positions_(0) = 0.5;     // Joint 1
        init_joint_positions_(1) = -0.7854; // Joint 2
        init_joint_positions_(2) = 0.0;     // Joint 3
        init_joint_positions_(3) = 1.3962;  // Joint 4
        init_joint_positions_(4) = 0.0;     // Joint 5
        init_joint_positions_(5) = 0.6109;  // Joint 6
        init_joint_positions_(6) = 0.0;     // Joint 7

        prev_dqd.resize(joint_positions_.data.size());
	prev_dqd.data.setZero(); 
	
	if (cmd_interface_ == "effort") {
        // Update KDLRobot object with initial joint positions
        robot_->update(toStdVector(init_joint_positions_.data), std::vector<double>(nj, 0.0)); }
	else {
        // Update KDLRobot object with initial joint positions
        robot_->update(toStdVector(joint_positions_.data), std::vector<double>(nj, 0.0)); }
        
        // Add end-effector frame if necessary
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);

        // Compute EE frame based on initial joint positions
        init_cart_pose_ = robot_->getEEFrame();

        // Initialize controller with the robot model
        controller_ = std::make_shared<KDLController>(*robot_);

        // EE's trajectory initial position
        Eigen::Vector3d init_position(init_cart_pose_.p.data);

        // EE's trajectory end position (just opposite y)
        Eigen::Vector3d end_position;
        end_position << init_position[0], -init_position[1], init_position[2];

        // Plan trajectory based on trajectory_index_
        double traj_duration = 10, acc_duration = 5, trajRadius = 0.1;

        // Create planners
        planners_.emplace_back(traj_duration, acc_duration, init_position, end_position); // Linear Trapezoidal
        planners_.back().setProfileType(ProfileType::TRAPEZOIDAL);
        planners_.back().setTrajectoryType(TrajectoryType::LINEAR);

        planners_.emplace_back(traj_duration, acc_duration, init_position, end_position); // Linear Cubic
        planners_.back().setProfileType(ProfileType::CUBIC);
        planners_.back().setTrajectoryType(TrajectoryType::LINEAR);

        planners_.emplace_back(traj_duration, init_position, trajRadius); // Circular Trapezoidal
        planners_.back().setProfileType(ProfileType::TRAPEZOIDAL);
        planners_.back().setTrajectoryType(TrajectoryType::CIRCULAR);

        planners_.emplace_back(traj_duration, init_position, trajRadius); // Circular Cubic
        planners_.back().setProfileType(ProfileType::CUBIC);
        planners_.back().setTrajectoryType(TrajectoryType::CIRCULAR);

        // Select the planner based on trajectory_index_
        if (trajectory_index_ >= 0 && trajectory_index_ < static_cast<int>(planners_.size()))
        {
            planner_ = planners_[trajectory_index_];
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid trajectory index provided. Please provide a trajectory index between 0 and %ld.", planners_.size() - 1);
            return;
        }
        
        // Rotation from the end-effector to the camera
        R_ee_to_camera <<
            0, -1,  0,
            1,  0,  0,
            0,  0,  1;

        // Create the publisher for commands based on the interface
        if (cmd_interface_ == "position")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        }
        else if (cmd_interface_ == "velocity")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
        }
        else if (cmd_interface_ == "effort")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
            torque_plot_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/torque_plot", 10); //topic for torque
        }

        // Create the timer for the control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Iiwa_pub_sub::cmd_publisher, this)
        );

        // Initialize the desired commands
        desired_commands_.resize(nj, 0.0);

        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
    }

private:
    void cmd_publisher()
    {
        iteration_ += 1;

        if (task_ == "positioning" && cmd_interface_ == "velocity")
        {
            // Call the function to compute positioning
            compute_positioning();      
        }
        else if (task_ == "look-at-point" && cmd_interface_ == "velocity")
        {
            // Call the function to compute the control law
            compute_look_at_point_control();
        }
        else if (cmd_interface_ == "effort") 
        {
            if (!posizione_iniziale_raggiunta_)
            {
                // Pre-trajectory
                return_to_initial_position();
            }
            else
            {
                // Execute trajectory
                effort_control();
            }
        }
    }

    void compute_positioning()
    {
        // Check that the ArUco pose is available
        if (!aruco_pose_available_)
        {
            RCLCPP_WARN(this->get_logger(), "ArUco pose not available.");
            return;
        }

        desired_frame_operations(); // Desired Frame
        RCLCPP_INFO(this->get_logger(), "==== Executing Positioning Task ====");

        // Calculate the positional error
        Eigen::Vector3d pos_error = computeLinearError(
            Eigen::Vector3d(desired_frame_.p.data),
            Eigen::Vector3d(robot_->getEEFrame().p.data)
        );

        // Calculate the orientation error
        Eigen::Vector3d ori_error = computeOrientationError(
            toEigen(desired_frame_.M),
            toEigen(robot_->getEEFrame().M)
        );

        // Create the Cartesian velocity vector for position and orientation
        double Kp_pos = 10;
        double Kp_ori = 3;
        Vector6d cartvel;
        cartvel << Kp_pos * pos_error(0),
                   Kp_pos * pos_error(1),
                   Kp_pos * pos_error(2),
                   Kp_ori * ori_error(0),
                   Kp_ori * ori_error(1),
                   Kp_ori * ori_error(2);

        Eigen::VectorXd q_dot = pseudoinverse(robot_->getEEJacobian().data) * cartvel; // q_dot

        double max_speed = 0.5; // Limit joint speeds
        for (size_t i = 0; i < q_dot.size(); ++i)
        {
            if (q_dot(i) > max_speed) q_dot(i) = max_speed;
            else if (q_dot(i) < -max_speed) q_dot(i) = -max_speed;
        }

        // Update the commands for the joint velocities
        for (size_t i = 0; i < joint_velocities_cmd_.rows(); ++i)
        {
            joint_velocities_cmd_(i) = q_dot(i);
            desired_commands_[i] = joint_velocities_cmd_(i);
        }

        // Update the robot's KDL structure
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Publish the command
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < msg->position.size(); i++)
        {
            joint_positions_.data[i] = msg->position[i];
            joint_velocities_.data[i] = msg->velocity[i];
        }
    }

    void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // ArUco pose
        aruco_pose_.p.data[0] = msg->pose.position.x;
        aruco_pose_.p.data[1] = msg->pose.position.y;
        aruco_pose_.p.data[2] = msg->pose.position.z;

        KDL::Rotation rot;
        aruco_pose_.M = rot.Quaternion(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        // Indicate that the ArUco pose is available
        aruco_pose_available_ = true;
        q_init_ = robot_->getJntValues();
    }

void desired_frame_operations()
{
    // Frame corrente del fine effettore
    KDL::Frame T_c_w = robot_->getEEFrame();
    Eigen::Vector3d Pc_w = toEigen(T_c_w.p); // Position of the camera in the World frame

    Eigen::Matrix3d R_ee = toEigen(T_c_w.M);   // EE rotation in the World frame
    Eigen::Matrix3d Rc   = R_ee * R_ee_to_camera; // Camera rotation in the World frame

    Eigen::Vector3d Pos_c = toEigen(aruco_pose_.p);

    Eigen::Vector3d Po_w = Rc * Pos_c + Pc_w; // Transform the tag position into the World frame

    Po_w.x() += 0.9; // x offset

    // Transform the ArUco orientation into the World frame
    Eigen::Matrix3d R_aruco_c = toEigen(aruco_pose_.M); // Tag rotation in the camera frame
    Eigen::Matrix3d R_aruco_w = Rc * R_aruco_c;         // Tag rotation in the World frame

    // Create a rotation of 180° around the X-axis
    Eigen::Matrix3d R_x_180;
    R_x_180 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // 180° rotation around X

    // Apply the rotation of 180° around X to the current orientation
    Eigen::Matrix3d R_rotated = R_aruco_w * R_x_180;

    // Apply an additional +360° rotation around Z
    Eigen::Matrix3d R_yaw_360;
    R_yaw_360 = Eigen::AngleAxisd(2 * M_PI, Eigen::Vector3d::UnitZ()); // 360° rotation around Z

    R_rotated = R_rotated * R_yaw_360;

    // Apply an additional 90° rotation around Z
    Eigen::Matrix3d R_z_90;
    R_z_90 = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()); // 90° rotation around Z

    Eigen::Matrix3d R_desired = R_rotated * R_z_90;

    // Assign the desired rotation to the KDL frame
    KDL::Rotation desired_rotation = toKDL(R_desired);

    KDL::Frame desired_frame_world;
    desired_frame_world.p = KDL::Vector(Po_w.x(), Po_w.y(), Po_w.z());
    desired_frame_world.M = desired_rotation;

    desired_frame_ = desired_frame_world;

    // Log the desired position
    /*RCLCPP_INFO(
        this->get_logger(),
        "Desired position with offset (Po_w): [%.3f, %.3f, %.3f]",
        desired_frame_.p[0], desired_frame_.p[1], desired_frame_.p[2]
    ); 

    // Log the desired orientation
    double roll, pitch, yaw;
    desired_frame_.M.GetRPY(roll, pitch, yaw);
    RCLCPP_INFO(
        this->get_logger(),
        "Desired orientation (RPY): roll=%.3f, pitch=%.3f, yaw=%.3f",
        roll, pitch, yaw
    ); */
}

    void compute_look_at_point_control()
    {
        // Check that the ArUco pose is available
        if (!aruco_pose_available_)
        {
            RCLCPP_WARN(this->get_logger(), "ArUco pose is not available.");
            return;
        }

        // Update the robot's KDL structure with the current positions and velocities
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Object position in the World frame
        Eigen::Vector3d Po_c = toEigen(aruco_pose_.p);

        // Calculate the normalized "look-at" vector
        Eigen::Vector3d s = Po_c.normalized();

        // Calculate the antisymmetric matrix S(s), representing the cross product
        Eigen::Matrix3d S_s;
        S_s << 0,     -s(2),  s(1),
               s(2),   0,    -s(0),
              -s(1),  s(0),   0;

        // Calculate the mapping matrix L(s)
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3, 6> Ls;
        Ls.block<3,3>(0,0) = -(1.0 / Po_c.norm()) * (I - s * s.transpose());
        Ls.block<3,3>(0,3) = S_s;

        // Calculate the transformation matrix R
        Eigen::Matrix<double, 6, 6> R;
        R.setZero();
        Eigen::Matrix3d Rc = toEigen(robot_->getEEFrame().M) * R_ee_to_camera;
        R.block<3,3>(0,0) = Rc; // Current rotation
        R.block<3,3>(3,3) = Rc;

        // Calculate the overall matrix L
        Eigen::Matrix<double, 3, 6> L = Ls * R;

        // Obtain the end-effector Jacobian
        Eigen::MatrixXd Jc = robot_->getEEJacobian().data;

        // Calculate the combined matrix L * Jc
        Eigen::MatrixXd LJc = L * Jc;

        // Calculate the error between the desired vector and the current one
        Eigen::Vector3d sd(0.0, 0.0, 1.0); // Desired vector (the camera's Z-axis)
        Eigen::Vector3d e = sd.cross(s);
        double error_norm = e.norm();

        // Calculate the pseudoinverse of (L J_c)
        Eigen::MatrixXd LJc_pinv = pseudoinverse(LJc);

        // Calculate the null-space term to handle redundancy
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(robot_->getNrJnts(), robot_->getNrJnts()) - LJc_pinv * LJc;
        double k_0 = 5; // Proportional gain for the null-space term

        // Obtain the current and initial joint values
        Eigen::VectorXd q_current = robot_->getJntValues();

        Eigen::VectorXd q_dot_0 = -k_0 * (q_current - q_init_);

        // Calculate the final joint commands
        double k = 2; // Proportional gain
        Eigen::VectorXd q_dot = k * LJc_pinv * e + N * q_dot_0;

        // Limit the maximum joint speed
        double max_speed = 0.5;
        for (size_t i = 0; i < q_dot.size(); ++i)
        {
            if (q_dot(i) > max_speed) q_dot(i) = max_speed;
            else if (q_dot(i) < -max_speed) q_dot(i) = -max_speed;
        }

        // Save and publish the calculated commands
        for (size_t i = 0; i < q_dot.size(); ++i)
        {
            desired_commands_[i] = q_dot(i);
            RCLCPP_INFO(this->get_logger(), "q_dot[%zu]: %.6f", i, q_dot(i));
        }

        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

        // Debug: print useful information
        /*
        RCLCPP_INFO(this->get_logger(), "Camera position (Pc_w): [%.3f, %.3f, %.3f]", Pc_w.x(), Pc_w.y(), Pc_w.z());
        RCLCPP_INFO(this->get_logger(), "Object position in the world (Po_w): [%.3f, %.3f, %.3f]", Po_w.x(), Po_w.y(), Po_w.z());
        RCLCPP_INFO(this->get_logger(), "Look-At Vector (s): [x: %.3f, y: %.3f, z: %.3f]", s.x(), s.y(), s.z());
        RCLCPP_INFO(this->get_logger(), "Error (e): [x: %.3f, y: %.3f, z: %.3f]", e.x(), e.y(), e.z());
        RCLCPP_INFO(this->get_logger(), "Error norm: %.6f", error_norm);
        */
    }

    void return_to_initial_position() {
    	// Calculate the joint error between the desired and current positions
    	KDL::JntArray q_error(init_joint_positions_.rows());
  	  for (unsigned int i = 0; i < init_joint_positions_.rows(); ++i) {
 	       q_error(i) = init_joint_positions_(i) - joint_positions_(i);
 	   }

 	// Control gains
  	double Kp = 100.0; // Proportional gain
   	double Kd = 20.0;  // Derivative gain
   	double soglia_errore_ = 0.2;

   	// Calculate control torques
  	Eigen::VectorXd torques = Kp * q_error.data - Kd * joint_velocities_.data;

 	// Gravity compensation
  	robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
   	Eigen::VectorXd gravity = robot_->getGravity();
   	torques += gravity;

  	// Assign calculated torques to joint effort commands
  	  for (int i = 0; i < torques.size(); ++i) {
  	      joint_efforts_cmd_(i) = torques(i);
  	      desired_commands_[i] = joint_efforts_cmd_(i);
 	   }

 	// Create and publish the message
   	std_msgs::msg::Float64MultiArray cmd_msg;
  	cmd_msg.data = desired_commands_;
   	cmdPublisher_->publish(cmd_msg);
  	RCLCPP_INFO_ONCE(this->get_logger(), "Returning manipulator to initial position...");

  	// Check if the initial position has been reached
  	  if (q_error.data.norm() <= soglia_errore_) {
  	      posizione_iniziale_raggiunta_ = true;
  	      RCLCPP_INFO(this->get_logger(), "Initial position reached.");
   	     RCLCPP_INFO(this->get_logger(), "Starting trajectory execution...");
  	  }
     }

     void effort_control() {
    	// Define trajectory
   	double total_time = 10;
   	int trajectory_len = 150;
   	int loop_rate = trajectory_len / total_time;
  	double dt = 1.0 / loop_rate;
  	t_ += dt;

   	// Update the KDLRobot structure with current states
  	robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
  	

   	 if (t_ <= total_time) {
            // Retrieve the trajectory point
            trajectory_point p = planner_.compute_trajectory(t_);

            // Compute EE frame
            KDL::Frame cartpos = robot_->getEEFrame();
            
            //Desired Frame
            desired_frame_operations();
            
            // Compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(desired_frame_.M));

            if (control_space_ == "operational_space")
            {
                // Effort control using idCntr_OSpace
                
                // Desired position
                KDL::Frame desPos;
                desPos.p = toKDL(p.pos);
                desPos.M = cartpos.M; //desired_frame_.M

	    	if (trajectory_index_ == 2 || trajectory_index_ == 3){
		desPos.M = desired_frame_.M;}
                // Desired velocity
                KDL::Twist desVel;
                desVel.vel = toKDL(p.vel);
                desVel.rot = KDL::Vector(0.0, 0.0, 0.0);

                // Desired acceleration
                KDL::Twist desAcc;
                desAcc.vel = toKDL(p.acc);
                desAcc.rot = KDL::Vector(0.0, 0.0, 0.0);

                // Control gains
                double Kpp = 350;
                double Kpo = 150;
                double Kdp = 100;
                double Kdo = 100;

                // Compute torques using idCntr in Operational space
                Eigen::VectorXd torques = controller_->idCntr_OSpace(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

                // Assign computed torques to joint effort commands
                for (int i = 0; i < torques.size(); ++i)
                {
                    joint_efforts_cmd_(i) = torques(i);
                    RCLCPP_INFO(this->get_logger(), "torques[%u]: %.6f", i, torques(i));
                }

	    	// === Pubblicazione per rqt_plot ===
	    	std_msgs::msg::Float64MultiArray torque_msg;
	    	torque_msg.data.resize(torques.size());
	    	for (int i = 0; i < torques.size(); ++i)
	    	{
			torque_msg.data[i] = torques(i);
	    	}
	    	torque_plot_publisher_->publish(torque_msg);
    
            }
            else if (control_space_ == "joint_space")
            {
            	//Effort control using idCntr_JSpace
            	
	    	//Calcolo della posizione, velocità e accelerazione desiderata dei giunti
	    	unsigned int nj = robot_->getNrJnts();
		KDL::JntArray qd(nj), dqd(nj), ddqd(nj);

	    	for (unsigned int i = 0; i < joint_positions_.data.size(); ++i)
	    	{
			// Posizione desiderata tramite IK
			KDL::Frame nextFrame;
			nextFrame.M = desired_frame_.M;
			
			Eigen::Vector3d error_scaled = 1 * error;
			nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(error_scaled)) * dt;
			
			robot_->getInverseKinematics(nextFrame, qd);

			// Velocità desiderata tramite Jacobiano inverso
			Vector6d cartvel;
			cartvel << p.vel + 5 * error, 15 * o_error;
			dqd.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;

			// Accelerazione desiderata come derivata della velocità
			if (iteration_ == 0)
			{
			    ddqd.data.setZero(); // Prima iterazione
			}
			else
			{
			    ddqd(i) = (dqd(i) - prev_dqd(i)) / dt; // Derivata numerica
			}
	    	}

	    	// Aggiorna lo stato precedente
	    	prev_dqd = dqd;
            	
            	// Control gains
            	double Kp = 150;
            	double Kd = 20; 
            	
            	//Compute torques using idCntr_OSpace
            	Eigen::VectorXd torques = controller_->idCntr_JSpace(qd, dqd, ddqd, Kp, Kd);

                // Assign computed torques to joint effort commands
                for (int i = 0; i < torques.size(); ++i)
                {
                    joint_efforts_cmd_(i) = torques(i);
                    RCLCPP_INFO(this->get_logger(), "torques[%u]: %.6f", i, torques(i));
                }
	    	// === Pubblicazione per rqt_plot ===
	    	std_msgs::msg::Float64MultiArray torque_msg;
	    	torque_msg.data.resize(torques.size());
	    	for (int i = 0; i < torques.size(); ++i)
	    	{
			torque_msg.data[i] = torques(i);
	    	}
	    	torque_plot_publisher_->publish(torque_msg);                            	            
            }

            // Set joint effort commands
            for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i)
                {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }

            // Create message and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

   	 } else {
    	    if (!maintenance_message_printed) {
    	        RCLCPP_INFO(this->get_logger(), "Maintenance mode...");
    	        maintenance_message_printed = true;
   	     }
    	    maintanance_mode();
   	 }
     }

     void maintanance_mode() {
   	 // Continue using idCntr to maintain the final position
  	 robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

  	 // Get the current position of the manipulator
   	 KDL::Frame current_cart_pose = robot_->getEEFrame();

  	 // Desired position
  	 KDL::Frame desPos = current_cart_pose;

  	 // Desired velocity and acceleration set to zero
  	 KDL::Twist desVel;
  	 desVel.vel = KDL::Vector(0.0, 0.0, 0.0);
   	 desVel.rot = KDL::Vector(0.0, 0.0, 0.0);

  	 KDL::Twist desAcc;
   	 desAcc.vel = KDL::Vector(0.0, 0.0, 0.0);
   	 desAcc.rot = KDL::Vector(0.0, 0.0, 0.0);

    	// Control gains
    	double Kpp = 50;
    	double Kpo = 50;
    	double Kdp = 5;
    	double Kdo = 50;

    	// Compute torques using idCntr
    	Eigen::VectorXd torques = controller_->idCntr_OSpace(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

    	// Assign computed torques to joint effort commands
    	for (int i = 0; i < torques.size(); ++i) {
        	joint_efforts_cmd_(i) = torques(i);
        	desired_commands_[i] = joint_efforts_cmd_(i);
    	}

    	// Create and publish the message
    	std_msgs::msg::Float64MultiArray cmd_msg;
    	cmd_msg.data = desired_commands_;
    	cmdPublisher_->publish(cmd_msg);
     }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr subTimer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_plot_publisher_; //topic per i torque, solo durante traiettoria

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;

    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;

    int iteration_;
    bool joint_state_available_;
    bool aruco_pose_available_;
    double t_;
    std::string cmd_interface_;
    std::string task_;
    std::string control_space_;
    std::vector<KDLPlanner> planners_;
    std::shared_ptr<KDLController> controller_;    
    KDL::Frame init_cart_pose_;
    KDL::Frame aruco_pose_;
    KDL::Frame aruco_in_camera_;
    KDL::JntArray init_joint_positions_;
    KDL::JntArray prev_dqd; // Velocità dei giunti al passo precedente
    KDL::Frame desired_frame_;
    int trajectory_index_;
    Eigen::Matrix3d R_ee_to_camera;
    Eigen::VectorXd q_init_;
    bool posizione_iniziale_raggiunta_;
    bool maintenance_message_printed = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // Parse command-line arguments
    int trajectory_index = 0; // Default value
    if (argc > 1)
    {
        trajectory_index = std::atoi(argv[1]);
    }
    else
    {
        std::cout << "Please provide a trajectory index (0-3)." << std::endl;
        return 1;
    }
    auto node = std::make_shared<Iiwa_pub_sub>(trajectory_index);
    rclcpp::spin(node);        
    rclcpp::shutdown();
    return 1;
}

