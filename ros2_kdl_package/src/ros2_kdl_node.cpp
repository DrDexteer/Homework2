// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);

            declare_parameter("traj_type", "linear_polynomial"); // defaults to "linear_polynomial"
            get_parameter("traj_type", traj_type_);

            declare_parameter("control_type", "operative_space"); // Default: operative_space
            get_parameter("control_type", control_type_);




            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            if (!(traj_type_ == "linear_polynomial" || traj_type_ == "linear_trapezoidal" ||
                  traj_type_ == "circle_polynomial" || traj_type_ == "circle_trapezoidal"))
            {
                RCLCPP_INFO(get_logger(),"Selected trajectory type is not valid!"); return;
            }

            if (control_type_ != "joint_space" && control_type_ != "operative_space") {
                RCLCPP_ERROR(this->get_logger(), "Invalid control type specified: '%s'", control_type_.c_str());
                rclcpp::shutdown();
                return;
            }


            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
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

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_.resize(nj); joint_efforts_cmd_.data.setZero(); 
            qd_.resize(nj);
            dqd_.resize(nj);
            ddqd_.resize(nj);
            dqd_old_.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2]+0.1;

            // Plan trajectory
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, radius=0.1;
            linear_planner = KDLPlanner(traj_duration, init_position, end_position);
            circle_planner = KDLPlanner(traj_duration, init_position, radius);

    
            trajectory_point p;

            if(traj_type_ == "linear_polynomial"){
                p = linear_planner.compute_trajectory_linear(t);
            }else if(traj_type_ == "linear_trapezoidal"){
                p = linear_planner.compute_trajectory_linear(t, acc_duration);
            }else if(traj_type_ == "circle_polynomial"){
                p = circle_planner.compute_trajectory_circle(t);
            }else if(traj_type_ == "circle_trapezoidal"){
                p = circle_planner.compute_trajectory_circle(t, acc_duration);
            }

            
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                for (long int i = 0; i < joint_efforts_.data.size(); ++i) 
                {
                    desired_commands_[i] = joint_efforts_(i);
                }  
                

                
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 3; // 
            int trajectory_len = 300; // 
            double acc_duration = 0.5;
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            KDLController controller_(*robot_); 
            
            if(true)
            {

                if (t_ <= total_time){                
                    if(traj_type_ == "linear_polynomial"){
                        p = linear_planner.compute_trajectory_linear(t_);
                    }else if(traj_type_ == "linear_trapezoidal"){
                        p = linear_planner.compute_trajectory_linear(t_, acc_duration);
                    }else if(traj_type_ == "circle_polynomial"){
                        p = circle_planner.compute_trajectory_circle(t_);
                    }else if(traj_type_ == "circle_trapezoidal"){
                        p = circle_planner.compute_trajectory_circle(t_, acc_duration);
                    } 
                }


                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;  
                         

                //Compute desired Frame
                KDL::Frame des_cartpos;
                des_cartpos.M = cartpos.M;  
                des_cartpos.p = toKDL(p.pos);  

                // Desired frame velocity 
                KDL::Twist des_cartvel;
                des_cartvel.vel = toKDL(p.vel);  

                // Desired frame acceleration 
                KDL::Twist des_cartacc;
                des_cartacc.vel = toKDL(p.acc); 

                

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){

                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_cmd_.data = joint_positions_.data + joint_velocities_.data*dt;
                }
                else if(cmd_interface_ == "effort")
                {
                    if (control_type_ == "joint_space") {
                        KDL::Frame eeFrame_d(init_cart_pose_.M,toKDL(p.pos));
                        robot_->getInverseKinematics(eeFrame_d, qd_);

                        KDL::Twist eeTwist_d(toKDL(p.vel),KDL::Vector::Zero());
                        robot_->getInverseKinematicsVel(eeTwist_d, dqd_);

                        ddqd_.data = (dqd_.data - dqd_old_.data) / dt;
                        dqd_old_ = dqd_;
                    } else if (control_type_ == "operative_space"){
                        KDL::Frame des_cartpos(init_cart_pose_.M, (toKDL(p.vel) + toKDL(0.1 * error)) * dt); 
                    
                        // Desired frame velocity 
                        KDL::Twist des_cartvel;
                        des_cartvel.vel = toKDL(p.vel+ 0.1 * error);  

                        // Desired frame acceleration 
                        KDL::Twist des_cartacc;
                        des_cartacc.vel = toKDL(p.acc); 

                    }               
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));


                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }else if(cmd_interface_ == "effort"){
                    if(control_type_ == "joint_space"){
                        desired_commands_ = toStdVector(controller_.idCntr(qd_, dqd_, ddqd_, kp_, kd_));
                    } else if (control_type_ == "operative_space") {                            
                        desired_commands_ = toStdVector(controller_.idCntr2(des_cartpos, des_cartvel, des_cartacc, kpp_, kpo_, kdp_, kdo_));
                    } 
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_accelerations_;
        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;
        KDL::JntArray joint_efforts_;
         
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        KDLPlanner linear_planner;
        KDLPlanner circle_planner;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        KDL::Frame init_cart_pose_;
        std::string traj_type_;
        std::string control_type_;

        KDL::JntArray qd_, dqd_, ddqd_, qd_old_, dqd_old_;
        double kp_=70, kd_=25;
        trajectory_point p;
        double kpp_=100.0;  
        double kpo_=30.0;  
        double kdp_=30.0;  
        double kdo_=25.0;


};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}