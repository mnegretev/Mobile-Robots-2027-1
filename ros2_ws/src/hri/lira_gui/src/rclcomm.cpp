#include "rclcomm.h"
#include <string>
#include <chrono>

RclComm::RclComm(): Node("justina_gui_node")
{
    std::cout << "Initializing RclComm node ..." << std::endl;
    this->current_arm_joints.resize(6);
    this->_pub_cmd_vel =  this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->_pub_traj = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
    this->_pub_tts_query = this->create_publisher<std_msgs::msg::String>("/tts_query",1);
    // auto timer_callback =
    // 	[this]() -> void {
    // 	    std::cout << "Testing callback" << std::endl;
    // 	};
    //_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RclComm::timer_callback, this));
    this->_sub_joint_states = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",1,std::bind(&RclComm::callback_joint_states,this,std::placeholders::_1));
    this->_clt_plan_path = this->create_client<nav_msgs::srv::GetPlan>("/path_planning/plan_path");
    this->_clt_smooth_path = this->create_client<navig_msgs::srv::ProcessPath>("/path_planning/smooth_path");
    this->_clt_ik_pose2pose = this->create_client<manip_msgs::srv::InverseKinematicsPose2Pose>("/manipulation/ik_pose2pose");
    this->_clt_fwd_kinematics = this->create_client<manip_msgs::srv::ForwardKinematics>("/manipulation/forward_kinematics");
    this->_clt_recog_obj  = this->create_client<pumas_vision_msgs::srv::RecognizeObject>("/vision/recognize_object");
    this->_clt_recog_objs = this->create_client<pumas_vision_msgs::srv::RecognizeObjects>("/vision/recognize_objects");
}

void RclComm::spin_some()
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
    RCLCPP_INFO(this->get_logger(), "LiraGUI->Waiting for plan path service to be available...");
    while(!this->_clt_plan_path->wait_for_service(std::chrono::milliseconds(500)))
    {
	if (!rclcpp::ok()) {
	    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the plan path service. Exiting.");
	    return 0;
	}
	RCLCPP_INFO(this->get_logger(), "service plan path not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "LiraGUI->Plan path service available. Trying to plan path...");
    auto result = this->_clt_plan_path->async_send_request(request);
    // Wait for the result.
    bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	path = result.get()->plan;
	RCLCPP_INFO(this->get_logger(), "LiraGUI.->Path planned successfully. Path with %lu points.", path.poses.size());
	return true;
    } else {
        RCLCPP_INFO(this->get_logger(), "LiraGUI.->Cannot plan path :'(");
	return false;
    }
    return true;
}

bool RclComm::call_smooth_path(nav_msgs::msg::Path& path, nav_msgs::msg::Path& smooth_path){
    auto request = std::make_shared<navig_msgs::srv::ProcessPath::Request>();
    request->path = path;
    RCLCPP_INFO(this->get_logger(), "LiraGUI->Waiting for smooth path service to be available...");
    int counter_timeout = 5;
    while(!this->_clt_smooth_path->wait_for_service(std::chrono::milliseconds(100)) && --counter_timeout > 0)
    {
        if (!rclcpp::ok()) 
	    return 0;
	//RCLCPP_INFO(this->get_logger(), "service smooth path not available, waiting again...");
    }
    if(counter_timeout <= 0){
        RCLCPP_INFO(this->get_logger(), "LiraGUI->Smooth path service is not available. ");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "LiraGUI->Smooth path service available. Trying to smooth path...");
    auto result = this->_clt_smooth_path->async_send_request(request);
    // Wait for the result.
    bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	smooth_path = result.get()->processed_path;
	RCLCPP_INFO(this->get_logger(), "LiraGUI.->Path smoothed successfully.");
	return true;
    } else {
        RCLCPP_INFO(this->get_logger(), "LiraGUI.->Cannot smooth path :'(");
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

  double time = 0.5 + 0.5*max_delta;
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
    req->initial_guess = this->current_arm_joints;
    //RCLCPP_INFO(this->get_logger(), "LiraGUI->Waiting for IK service to be available...");
    int counter_timeout = 5;
    while(!this->_clt_ik_pose2pose->wait_for_service(std::chrono::milliseconds(100)) && --counter_timeout > 0)
    {
	if (!rclcpp::ok()) {
	    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the IK service. Exiting.");
	    return 0;
	}
	RCLCPP_INFO(this->get_logger(), "service IK not available, waiting again...");
    }
    if(counter_timeout <= 0)
	return false;
    //RCLCPP_INFO(this->get_logger(), "LiraGUI->IK service available. Trying to solve IK...");
    auto response = this->_clt_ik_pose2pose->async_send_request(req);
    // Wait for the result.
    bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	Q = response.get()->q;
	//RCLCPP_INFO(this->get_logger(), "LiraGUI.->IK solved successfully");
    } else {
	RCLCPP_INFO(this->get_logger(), "LiraGUI.->Cannot solve IK :'(");
    }
    return success;
}

bool RclComm::call_fwd_kinematics(std::vector<double>& Q, double& x, double& y, double& z, double& roll, double& pitch, double& yaw){
    auto req = std::make_shared<manip_msgs::srv::ForwardKinematics::Request>();
    req->q = Q;
    int counter = 5;
    while(!this->_clt_fwd_kinematics->wait_for_service(std::chrono::milliseconds(20)) && --counter > 0)
    {
	if (!rclcpp::ok()) {
	    //RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the FK service. Exiting.");
	    return 0;
	}
	//RCLCPP_INFO(this->get_logger(), "service FK not available, waiting again...");
    }
    if(counter <= 0)
	return false;
    //RCLCPP_INFO(this->get_logger(), "LiraGUI->FK service available. Trying to get FK...");
    auto response = this->_clt_fwd_kinematics->async_send_request(req);
    // Wait for the result.
    bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	auto result = response.get();
        x = result->x;
	y = result->y;
	z = result->z;
	roll = result->roll;
	pitch = result->pitch;
	yaw = result->yaw;
	//RCLCPP_INFO(this->get_logger(), "LiraGUI.->FK calculated successfully");
    } else {
	//RCLCPP_INFO(this->get_logger(), "LiraGUI.->Cannot get FK :'(");
    }
    return success;
}

bool RclComm::call_recog_obj(std::string id){
    auto req = std::make_shared<pumas_vision_msgs::srv::RecognizeObject::Request>();
    req->id = id;
    int counter = 5;
    while(!this->_clt_recog_obj->wait_for_service(std::chrono::milliseconds(20)) && --counter > 0)
	if (!rclcpp::ok()) 
	    return 0;
    if(counter <= 0){
	RCLCPP_INFO(this->get_logger(), "LiraGUI->Service for recognizing object is not available");
	return false;
    }
    RCLCPP_INFO(this->get_logger(), "LiraGUI->Recognize object service available. Trying recognize %s", id);
    auto response = this->_clt_recog_obj->async_send_request(req);
    bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), response,std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	auto result = response.get();
	success &= result->obj.id == id;
	int img_x = result->obj.x;
	int img_y = result->obj.y;
	double x = result->obj.pose.position.x;
	double y = result->obj.pose.position.y;
	double z = result->obj.pose.position.z;
	RCLCPP_INFO(this->get_logger(), "LiraGUI.->Found %s at [%d,%d] with pose (%lf, %lf %lf)", id, img_x, img_y, x, y, z);
    } else 
	RCLCPP_INFO(this->get_logger(), "LiraGUI.->Cannot found %s", id);
    return success;
}

bool RclComm::call_recog_objs(std::vector<std::string>& found_objs){
    found_objs.clear();
    auto req = std::make_shared<pumas_vision_msgs::srv::RecognizeObjects::Request>();
    int counter = 5;
    while(!this->_clt_recog_objs->wait_for_service(std::chrono::milliseconds(20)) && --counter > 0)
	if (!rclcpp::ok()) 
	    return 0;
    if(counter <= 0){
	RCLCPP_INFO(this->get_logger(), "LiraGUI->Service for recognizing objects is not available");
	return false;
    }
    RCLCPP_INFO(this->get_logger(), "LiraGUI->Recognize objects service available. Trying recognize objects");
    auto response = this->_clt_recog_objs->async_send_request(req);
    bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), response,std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	auto result = response.get();
	for(size_t i=0; i<result->objs.size(); i++)
	    found_objs.push_back(result->objs[i].id);
	RCLCPP_INFO(this->get_logger(), "LiraGUI.->Found %d objects", result->objs.size());
    } else 
	RCLCPP_INFO(this->get_logger(), "LiraGUI.->Cannot found any object");
    return success;
}

void RclComm::publish_tts_query(std::string txt){
    std_msgs::msg::String msg;
    msg.data = txt;
    this->_pub_tts_query->publish(msg);
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
