#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>
#include <string> 
#include <vector> 
#include <fstream>
#include <map> 

#include "moveit_7sk/srv/go_to_named_pose.hpp"
#include "moveit_7sk/srv/go_to_pose.hpp"
#include "moveit_7sk/srv/set_gripper.hpp"
#include "moveit_7sk/srv/add_named_pose.hpp"


class MoveitAPINode : public rclcpp::Node 
{
public: 
    MoveitAPINode(): Node("Moveit_api"){
        
        this->declare_parameter("velocity_scaling", 0.5);
        this->declare_parameter("acceleration_scaling", 0.5);
        this->declare_parameter("arm_group", std::string("arm"));
        this->declare_parameter("gripper_group", std::string("gripper"));
        this->declare_parameter("named_poses_file", std::string("named_poses.yaml"));

        vel_  = this->get_parameter("velocity_scaling").as_double();
        acc_  = this->get_parameter("acceleration_scaling").as_double();
        arm_grp_      = this->get_parameter("arm_group").as_string();
        gripper_grp_  = this->get_parameter("gripper_group").as_string();
        named_poses_file_ = this->get_parameter("named_poses_file").as_string();
        

        //SERVICES 
        go_to_named_pose_srv_ = create_service<moveit_7sk::srv::GoToNamedPose>(
            "go_to_named_pose", 
            std::bind(&MoveitAPINode::go_to_named_pose_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        go_to_pose_srv_ = create_service<moveit_7sk::srv::GoToPose>(
            "go_to_pose", 
            std::bind(&MoveitAPINode::go_to_pose_callback, this, std::placeholders:: _1, std::placeholders:: _2)
        );
        set_gripper_srv_ = create_service<moveit_7sk::srv::SetGripper>(
            "set_gripper", 
            std::bind(&MoveitAPINode::set_gripper_callback, 
            this, std::placeholders::_1, std::placeholders:: _2)
        );
    }

    void go_to_named_pose_callback(const std::shared_ptr<moveit_7sk::srv::GoToNamedPose::Request> request, std::shared_ptr<moveit_7sk::srv::GoToNamedPose::Response> response){
        move_group_arm_->setNamedTarget(request->pose_name); 

        moveit::planning_interface::MoveGroupInterface::Plan plan; 
        bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); 
        if(success){ 
            move_group_arm_->execute(plan); 
            response->success = true; 
            response->message = "Moved to: " + request->pose_name; 
        }else{
            response->success = false ; 
            response->message = "Planning failed for: " + request->pose_name; 
        }
    }

    void init(){
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), arm_grp_
        );
        move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), gripper_grp_
        );

        move_group_arm_->setPlanningPipelineId("ompl");
        move_group_arm_->setPlannerId("RRTConnect");
        move_group_arm_->setPlanningTime(10.0);
        move_group_arm_->setNumPlanningAttempts(100);
        move_group_arm_->setGoalPositionTolerance(0.005);
        // move_group_arm_->setGoalOrientationTolerance(1.00);

        move_group_arm_->setMaxVelocityScalingFactor(vel_);
        move_group_arm_->setMaxAccelerationScalingFactor(acc_);
        move_group_gripper_->setMaxVelocityScalingFactor(vel_);
        move_group_gripper_->setMaxAccelerationScalingFactor(acc_);
    }


    void set_gripper_callback(const std::shared_ptr<moveit_7sk::srv::SetGripper::Request> request,std::shared_ptr<moveit_7sk::srv::SetGripper::Response> response){
        move_group_gripper_->setJointValueTarget("gripper_left_joint" , request->value);

        moveit::planning_interface::MoveGroupInterface::Plan plan; 
        bool success = (move_group_gripper_->plan(plan)== moveit::core::MoveItErrorCode::SUCCESS); 

        if(success){
            move_group_gripper_->execute(plan); 
            response->success = true ; 
            response->message = "Gripper Position set successfully";
        } else{
            response->success = false; 
            response->message = "Planning failed";
        }
    }   

    void go_to_pose_callback(const std::shared_ptr<moveit_7sk::srv::GoToPose::Request> request, std::shared_ptr<moveit_7sk::srv::GoToPose::Response> response){
        // tf2::Quaternion q;
        // q.setRPY(request->roll, request->pitch, request->yaw);

        // geometry_msgs::msg::Pose target_pose;
        // target_pose.position.x = request->x;
        // target_pose.position.y = request->y;
        // target_pose.position.z = request->z;
        // target_pose.orientation = tf2::toMsg(q);

        // move_group_arm_->setPoseTarget(target_pose); 
        move_group_arm_->setPositionTarget(request->x, request->y, request->z);
        moveit::planning_interface::MoveGroupInterface::Plan plan; 
        bool success = (move_group_arm_->plan(plan)== moveit::core::MoveItErrorCode::SUCCESS); 

        if(success){
            move_group_arm_->execute(plan); 
            response->success = true; 
            response->message = "Planning is Successful";
        }else{
            response->success = false; 
            response->message = "Planning Failed";
        }
    }

private:
  double vel_;
  double acc_;
  std::string arm_grp_;
  std::string gripper_grp_;
  std::string named_poses_file_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
  rclcpp::Service<moveit_7sk::srv::GoToNamedPose>::SharedPtr go_to_named_pose_srv_;
  rclcpp::Service<moveit_7sk::srv::GoToPose>::SharedPtr go_to_pose_srv_;
  rclcpp::Service<moveit_7sk::srv::SetGripper>::SharedPtr set_gripper_srv_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitAPINode>();
    node->init();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}