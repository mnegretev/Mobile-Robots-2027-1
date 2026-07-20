#include "rclcomm.h"
#include <string>
#include <chrono>

RclComm::RclComm(): Node("justina_gui_node")
{
    std::cout << "Initializing RclComm node ..." << std::endl;
    this->current_arm_joints.resize(6);
    this->_pub_cmd_vel =  this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->_pub_traj = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
    // auto timer_callback =
    // 	[this]() -> void {
    // 	    std::cout << "Testing callback" << std::endl;
    // 	};
    //_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RclComm::timer_callback, this));
    this->_sub_joint_states = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",1,std::bind(&RclComm::callback_joint_states,this,std::placeholders::_1));
    this->_clt_plan_path = this->create_client<nav_msgs::srv::GetPlan>("/path_planning/plan_path");
    this->_clt_ik_pose2pose = this->create_client<manip_msgs::srv::InverseKinematicsPose2Pose>("/manipulation/ik_pose2pose");
}

void RclComm::spin_once()
{
  if(rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
  }
}

void RclComm::timer_callback()
{
    std::cout << "Testing callback" << std::endl;
}

void RclComm::callback_joint_states(const sensor_msgs::msg::JointState &msg)
{
  if(msg.name.size() == 7)
    for(int i=0; i< 6; i++)
      this->current_arm_joints[i] = msg.position[i+1];
}

void RclComm::publish_cmd_vel(double linear_x, double linear_y, double angular)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z =  angular;
    this->_pub_cmd_vel->publish(msg);
}

void RclComm::publish_cmd_vel(double linear_x, double angular)
{
    this->publish_cmd_vel(linear_x, 0, angular);
}

void RclComm::start_publishing_cmd_vel(double linear_x, double linear_y, double angular)
{
    _cmd_vel.linear.x = linear_x;
    _cmd_vel.linear.y = linear_y;
    _cmd_vel.angular.z = angular;
    _publishing_cmd_vel = true;
}

void RclComm::start_publishing_cmd_vel(double linear_x, double angular)
{
    this->start_publishing_cmd_vel(linear_x, 0, angular);
}

void RclComm::stop_publishing_cmd_vel()
{
    _cmd_vel.linear.x = 0;
    _cmd_vel.linear.y = 0;
    _cmd_vel.angular.z = 0;
    _publishing_cmd_vel = false;
}

bool RclComm::call_plan_path(double start_x, double start_y, double goal_x, double goal_y, nav_msgs::msg::Path& path)
{
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start.pose.position.x = start_x;
    request->start.pose.position.y = start_y;
    request->goal.pose.position.x = goal_x;
    request->goal.pose.position.y = goal_y;
    std::cout << "LiraGUI->Waiting for plan path service to be available..." << std::endl;
    while(!this->_clt_plan_path->wait_for_service(std::chrono::milliseconds(500)))
    {
	if (!rclcpp::ok()) {
	    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the plan path service. Exiting.");
	    return 0;
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service plan path not available, waiting again...");
    }
    std::cout << "LiraGUI->Plan path service available. Trying to plan path..." << std::endl;
    auto result = this->_clt_plan_path->async_send_request(request);
    // Wait for the result.
    bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	path = result.get()->plan;
	std::cout << "LiraGUI.->Path planned successfully. Path with " << path.poses.size() << " points." << std::endl;
	return true;
    } else {
	std::cout << "LiraGUI.->Cannot plan path :'(" << std::endl;
	return false;
    }
    return true;
}

void RclComm::publish_arm_joint_traj(std::vector<double> Q)
{
  double max_delta = -1;
  if(Q.size() != 6) return;
  for(int i=0; i< 6; i++)
    if(fabs(Q[i] - this->current_arm_joints[i]) > max_delta)
      max_delta = fabs(Q[i] - this->current_arm_joints[i]);

  double time = 0.5 + 1.0*max_delta;
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = this->get_clock()->now();
  msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions = {Q[0], Q[1], Q[2], Q[3], Q[4], Q[5]};
  p.time_from_start.sec = (int)time;
  p.time_from_start.nanosec = (int)(time*1000000000);
  msg.points.push_back(p);
  this->_pub_traj->publish(msg);
}

bool RclComm::call_ik_pose2pose(double x, double y, double z, double roll, double pitch, double yaw, std::vector<double>& Q){
    auto req = std::make_shared<manip_msgs::srv::InverseKinematicsPose2Pose::Request>();
    req->x = x;
    req->y = y;
    req->z = z;
    req->roll = roll;
    req->pitch = pitch;
    req->yaw = yaw;
    req->initial_guess = {0,0,0,0,0,0};
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiraGUI->Waiting for IK service to be available...");
    while(!this->_clt_ik_pose2pose->wait_for_service(std::chrono::milliseconds(500)))
    {
	if (!rclcpp::ok()) {
	    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the IK service. Exiting.");
	    return 0;
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service IK not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiraGUI->IK service available. Trying to solve IK...");
    auto response = this->_clt_ik_pose2pose->async_send_request(req);
    // Wait for the result.
    bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	Q = response.get()->q;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiraGUI.->IK solved successfully");
	return true;
    } else {
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiraGUI.->Cannot solve IK :'(");
	return false;
    }
    return true;
}

// void RclComm::recv_callback(const std_msgs::msg::String &msg)
// {
//     emit emitTopicData("pub send a msgs:" + QString::fromStdString(msg.data));
// }

// // spin
// void RclComm::sping()
// {
//     rclcpp::spin_some(_node);
// }
