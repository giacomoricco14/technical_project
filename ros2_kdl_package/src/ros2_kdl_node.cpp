#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <algorithm>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "std_msgs/msg/empty.hpp"

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
            declare_parameter("cmd_interface", "position"); 
            declare_parameter("traj_duration", 5.0);
            declare_parameter("acc_duration", 1.0);
            declare_parameter("Kp", 1.5);
            declare_parameter("end_position", std::vector<double>{0.13, 0.0, 0.6});

            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("traj_duration", traj_duration);
            get_parameter("acc_duration", acc_duration);
            get_parameter("Kp", Kp);
            get_parameter("end_position", end_position);

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 
            start_task_ = false;

            syncSubscriber_ = this->create_subscription<std_msgs::msg::Bool>(
                "/start_task", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    if(msg->data && this->motion_state_ == WAITING_FOR_BOT) {
                        this->start_task_ = true;
                    }
                });

            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "/robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) { rclcpp::shutdown(); }
                RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            KDL::Tree robot_tree;
            kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree);
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            while(!joint_state_available_){
                rclcpp::spin_some(node_handle_);
            }

            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            init_cart_pose_ = robot_->getEEFrame();
            
            drop_position_ = Eigen::Vector3d(init_cart_pose_.p.data);
            pick_position_ << end_position[0], end_position[1], end_position[2];
            
            place_waypoint_ << end_position[0], end_position[0], end_position[2]; 
            place_position_ << 0.5, 0.65, 0.5; 

            motion_state_ = WAITING_FOR_BOT;
            hold_time_ = 0.0;
            
            planner_ = KDLPlanner(traj_duration, acc_duration, drop_position_, pick_position_);
            if(cmd_interface_ == "position"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
            } else {
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            }
            gripperAttachPub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper_attach", 10);
            gripperDetachPub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper_detach", 10);
            timer_ = this->create_wall_timer(100ms, std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            RCLCPP_INFO(this->get_logger(), "KDL Ready! Waiting for camera trigger...");
        }

    private:
        enum MotionState { WAITING_FOR_BOT, GO_TO_PICK, PICKING, GO_TO_DROP, HOLD_TOP, PLACE_WAYPOINT, PLACE, DROPPING, RETURN_WAYPOINT, RETURN_HOME };
        MotionState motion_state_;
        double hold_time_;
        bool start_task_;
        
        Eigen::Vector3d drop_position_; 
        Eigen::Vector3d pick_position_; 
        Eigen::Vector3d place_waypoint_; 
        Eigen::Vector3d place_position_; 

        void cmd_publisher(){
            KDLController controller_(*robot_);
            double dt = 0.1; 
            t_ += dt;
            static int startup_counter = 0;
            if (startup_counter < 50) {
                std_msgs::msg::Empty drop_msg;
                gripperDetachPub_->publish(drop_msg);
                startup_counter++;
            }

            switch (motion_state_)
            {
            case WAITING_FOR_BOT:
                p_.pos = drop_position_;
                p_.vel.setZero();
                if (start_task_) {
                    RCLCPP_INFO(this->get_logger(), "Oggetto rilevato! Scendo a prenderlo...");
                    motion_state_ = GO_TO_PICK;
                    t_ = 0.0;
                    start_task_ = false; 
                }
                break;

            case GO_TO_PICK:
                p_ = planner_.linear_traj_trapezoidal(t_);
                if (t_ >= traj_duration) { motion_state_ = PICKING; hold_time_ = 0.0; }
                break;

            case PICKING:
                p_.pos = pick_position_; p_.vel.setZero(); hold_time_ += dt;
                
                if (hold_time_ >= 1.0 && hold_time_ < 1.1) {
                    std_msgs::msg::Empty grip_msg; 
                    gripperAttachPub_->publish(grip_msg);
                    RCLCPP_INFO(this->get_logger(), "*ZAK* Giunto Fisso Attivato!");
                }
                
                if (hold_time_ >= 2.0) { 
                    planner_ = KDLPlanner(traj_duration, acc_duration, pick_position_, drop_position_);
                    t_ = 0.0; motion_state_ = GO_TO_DROP; 
                    RCLCPP_INFO(this->get_logger(), "Inizio la salita...");
                }
                break;

            case GO_TO_DROP:
                p_ = planner_.linear_traj_trapezoidal(t_);
                if (t_ >= traj_duration) { 
                    motion_state_ = HOLD_TOP; 
                    hold_time_ = 0.0;
                }
                break;

            case HOLD_TOP:
                p_.pos = drop_position_; p_.vel.setZero(); hold_time_ += dt;
                if (hold_time_ >= 1.0) { 
                    planner_ = KDLPlanner(traj_duration, acc_duration, drop_position_, place_waypoint_);
                    t_ = 0.0; 
                    motion_state_ = PLACE_WAYPOINT; 
                    RCLCPP_INFO(this->get_logger(), "Mi sposto lateralmente (Fase 1/2)...");
                }
                break;

            case PLACE_WAYPOINT:
                p_ = planner_.linear_traj_trapezoidal(t_);
                if (t_ >= traj_duration) { 
                    planner_ = KDLPlanner(traj_duration, acc_duration, place_waypoint_, place_position_);
                    t_ = 0.0;
                    motion_state_ = PLACE; 
                    RCLCPP_INFO(this->get_logger(), "Mi sposto al deposito (Fase 2/2)...");
                }
                break;

            case PLACE:
                p_ = planner_.linear_traj_trapezoidal(t_);
                if (t_ >= traj_duration) { motion_state_ = DROPPING; hold_time_ = 0.0; }
                break;

            case DROPPING:
                p_.pos = place_position_; p_.vel.setZero(); hold_time_ += dt;
                
                if (hold_time_ >= 3.0 && hold_time_ < 3.1) {
                    std_msgs::msg::Empty grip_msg; 
                    gripperDetachPub_->publish(grip_msg);
                    RCLCPP_INFO(this->get_logger(), "*Swoosh* Cubo Rilasciato!");
                }

                if (hold_time_ >= 5.0) { 
                    planner_ = KDLPlanner(traj_duration, acc_duration, place_position_, place_waypoint_);
                    t_ = 0.0; motion_state_ = RETURN_WAYPOINT; 
                }
                break;

            case RETURN_WAYPOINT:
                p_ = planner_.linear_traj_trapezoidal(t_);
                if (t_ >= traj_duration) {
                    planner_ = KDLPlanner(traj_duration, acc_duration, place_waypoint_, drop_position_);
                    t_ = 0.0;
                    motion_state_ = RETURN_HOME;
                }
                break;

            case RETURN_HOME:
                p_ = planner_.linear_traj_trapezoidal(t_);
                if (t_ >= traj_duration) {
                    planner_ = KDLPlanner(traj_duration, acc_duration, drop_position_, pick_position_);
                    motion_state_ = WAITING_FOR_BOT;
                    RCLCPP_INFO(this->get_logger(), "Ciclo completato. In attesa del prossimo!");
                }
                break;
            }

            // ---------------------------------------------------

            KDL::Frame cartpos = robot_->getEEFrame();
            Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));

            if(cmd_interface_ == "position"){
                KDL::Frame nextFrame; 
                nextFrame.M = cartpos.M; 
                nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp*error))*dt; 
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                for (int i=0; i<7; i++) desired_commands_[i] = joint_positions_cmd_(i);
            }
            else if(cmd_interface_ == "velocity_ctrl" || cmd_interface_ == "velocity_ctrl_null"){
                Vector6d cartvel; 
                // QUI LA VERA MAGIA: 
                // Aggiungiamo la velocità feed-forward (omega_ff) e moltiplichiamo l'errore per Kp.
                // Ora polso e braccio viaggeranno in totale sincronia senza mai tremare!
                cartvel << p_.vel + Kp*error, o_error;
                
                joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                for (int i=0; i<7; i++) desired_commands_[i] = joint_velocities_cmd_(i);
            }

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }
            
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            unsigned int num_joints = std::min((size_t)sensor_msg.position.size(), (size_t)joint_positions_.data.size());            
            for (unsigned int i  = 0; i < num_joints; i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                if (i < sensor_msg.velocity.size()) joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        }
        
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripperAttachPub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripperDetachPub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr syncSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Node::SharedPtr node_handle_;
        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_, joint_velocities_, joint_positions_cmd_, joint_velocities_cmd_, joint_efforts_cmd_;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        trajectory_point p_;
        int iteration_;
        bool joint_state_available_;
        double t_, traj_duration, acc_duration, Kp;
        std::string cmd_interface_;
        std::vector<double> end_position;
        KDL::Frame init_cart_pose_;
};

int main( int argc, char** argv ) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}