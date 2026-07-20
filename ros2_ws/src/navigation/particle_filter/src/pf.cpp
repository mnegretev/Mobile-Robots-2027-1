/*
 * MOBILE ROBOTS - UNAM, FI, 2026-2
 * LOCALIZATION BY PARTICLE FILTERS
 *
 * Instructions:
 * Write the code necessary to implement localization by particle filters.
 * Modify only the sections marked with the TODO comment. 
 */

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "random_numbers/random_numbers.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "ParticleFilter_solved.h"

class ParticleFilterNode : public rclcpp::Node
{
public:

        ParticleFilterNode():Node("particle_filter")
    {
	RCLCPP_INFO(this->get_logger(), "INITIALIZING PARTICLE FILTER NODE - %s", FULL_NAME); 
	particles_N = this->declare_parameter<int>("N", 1000);
	min_x = this->declare_parameter<double>("min_x", -5.0);
	min_y = this->declare_parameter<double>("min_y", -4.0);
	min_a = this->declare_parameter<double>("min_a", -3.14);
	max_x = this->declare_parameter<double>("max_x",  12.0);
	max_y = this->declare_parameter<double>("max_y",  4.0);
	max_a = this->declare_parameter<double>("max_a",  3.14);
	laser_downsampling = this->declare_parameter<int>("ds", 10);
	sigma2_sensor     = this->declare_parameter<double>("s2s", 1.0);
	sigma2_movement   = this->declare_parameter<double>("s2m", 1.0);
	sigma2_resampling = this->declare_parameter<double>("s2r", 1.0);
	threshold_d = this->declare_parameter<double>("thr_d", 0.2);
	threshold_a = this->declare_parameter<double>("thr_a", 0.2);

	clt_static_map = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
	pub_particles = this->create_publisher<geometry_msgs::msg::PoseArray>("/particle_cloud", 1);
	sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1,std::bind(&ParticleFilterNode::scan_callback, this, std::placeholders::_1));
	sub_gz_pose=this->create_subscription<geometry_msgs::msg::PoseArray>("/gazebo/diff_base_with_xarm/pose",1,std::bind(&ParticleFilterNode::gz_pose_callback,this,std::placeholders::_1));
	tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
	tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    
    void spin()
    {
	/*
	 * Waiting for static map service to be available
	 */
	RCLCPP_INFO(this->get_logger(), "Waiting for static map");
	while(!this->clt_static_map->wait_for_service(std::chrono::seconds(1))){
	    if(!rclcpp::ok()){
		RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
		return;
	    }
	    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}
	/*
	 * Getting static map
	 */
	nav_msgs::msg::OccupancyGrid static_map;
	RCLCPP_INFO(this->get_logger(), "Static map service is now available. Getting static map.");
	nav_msgs::srv::GetMap::Request::SharedPtr req = std::make_shared<nav_msgs::srv::GetMap::Request>();
	rclcpp::Client<nav_msgs::srv::GetMap>::FutureAndRequestId result = this->clt_static_map->async_send_request(req);
	if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS){
	    static_map = result.get()->map;
	    RCLCPP_INFO(this->get_logger(), "Got map with size %d x %d", static_map.info.width, static_map.info.height);
	}else{
	    RCLCPP_INFO(this->get_logger(), "Cannot get static map. Aborting node. ");
	    return;
	}
	/*
	 * Waiting for the first real sensor reading
	 */
	while(this->real_scan.ranges.size() < 10){
	    RCLCPP_INFO(this->get_logger(), "Waiting for scan sensor readings...");
	    rclcpp::spin_some(this->get_node_base_interface());
	    rclcpp::sleep_for(std::chrono::seconds(1));
	}
	RCLCPP_INFO(this->get_logger(), "First sensor reading received with %lu ranges.", this->real_scan.ranges.size());
	this->simul_sensor_specs = this->real_scan;
	this->simul_sensor_specs.angle_increment *= this->laser_downsampling;
	/*
	 * Waiting for odom transform to be available
	 */
	RCLCPP_INFO(this->get_logger(), "Waiting for odom transform to be available...");
	while(rclcpp::ok()){
	    try {
		this->last_tf_odom = tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
		RCLCPP_INFO(this->get_logger(), "Odom transform is now available");
		break;
	    } catch (const tf2::TransformException & ex) {
		RCLCPP_INFO(this->get_logger(), "Could not get odom transform: %s", ex.what());
	    }
	}

	RCLCPP_INFO(this->get_logger(), "Initializing particle filter with parameters: ");
	RCLCPP_INFO(this->get_logger(), "N=%d\t downsampling=%d\t", particles_N, laser_downsampling);
	RCLCPP_INFO(this->get_logger(), "Sensor variance: %lf\tMovement variance: %lf\tResampling variance: %lf",
		    sigma2_sensor, sigma2_movement, sigma2_resampling);
	RCLCPP_INFO(this->get_logger(), "min_x=%lf\tmin_y=%lf\tmin_a=%lf", min_x, min_y, min_a);
	RCLCPP_INFO(this->get_logger(), "max_x=%lf\tmax_y=%lf\tmax_a=%lf", max_x, max_y, max_a);
	
	/*
	 * Initialize variables to estimate position by particle filters
	 */
	double delta_x, delta_y, delta_a;
	std::vector<geometry_msgs::msg::Pose2D> particles;        //A set of N particles
	std::vector<sensor_msgs::msg::LaserScan> simulated_scans; //A set of simulated laser readings, one scan per particle
	std::vector<double> similarities;                          //A set of similarities for each particle
	geometry_msgs::msg::TransformStamped tf_odom_to_map;

	particles = ParticleFilter::get_initial_distribution(particles_N, min_x, max_x, min_y, max_y, min_a, max_a);
	RCLCPP_INFO(this->get_logger(), "Publishing initial distribution...");
	this->pub_particles->publish(this->get_pose_array(particles));
	tf_odom_to_map = this->get_odom_to_map(particles);
	this->tf_broadcaster->sendTransform(tf_odom_to_map);
	int iteration = 0;
	while(rclcpp::ok()){
	    if(this->displacement_greater_than_threshold(delta_x, delta_y, delta_a)){
		RCLCPP_INFO(this->get_logger(), "Updating (%d) localization with delta_x=%lf\tdelta_y=%lf\tdelta_a=%lf",(++iteration), delta_x, delta_y, delta_a);
		/*
		 * This is the main algorithm of the particle filter:
		 * 1. Move all particles the odometry displacement and add noise
		 * 2. For each particle, simulate sensor readings
		 * 3. For each particle, get a similarity between the real and simulated reading
		 * 4. Use similarities as a prob distribution to make a resampling with replacement
		 */
		ParticleFilter::move_particles(particles, delta_x, delta_y, delta_a, this->sigma2_movement);
		simulated_scans = ParticleFilter::simulate_particle_scans(particles, static_map, this->simul_sensor_specs);
		similarities = ParticleFilter::get_particle_similarities(
			    simulated_scans, this->real_scan, this->laser_downsampling, this->sigma2_sensor);
		particles = ParticleFilter::resample_particles(particles, similarities, this->sigma2_resampling);
		/*
		 */

		this->pub_particles->publish(this->get_pose_array(particles));
		tf_odom_to_map = this->get_odom_to_map(particles);
		double estimated_x, estimated_y, estimated_a;
		this->get_estimated_pose(particles, estimated_x, estimated_y, estimated_a);
		RCLCPP_INFO(this->get_logger(), "Estimated pose: x=%lf\ty=%lf\ta=%lf",estimated_x, estimated_y, estimated_a);
		RCLCPP_INFO(this->get_logger(), "Groundtruth pose: x=%lf\ty=%lf\ta=%lf \n",this->groundtruth_x, this->groundtruth_y, this->groundtruth_a);
	    }
	    tf_odom_to_map.header.stamp = this->get_clock()->now();
	    this->tf_broadcaster->sendTransform(tf_odom_to_map);
	    rclcpp::spin_some(this->get_node_base_interface());
	    rclcpp::sleep_for(std::chrono::milliseconds(50));
	}
	
    }
    
private:
    int particles_N;
    double min_x;
    double min_y;
    double min_a;
    double max_x;
    double max_y;
    double max_a;
    int laser_downsampling;
    double sigma2_sensor;
    double sigma2_movement;
    double sigma2_resampling;
    double threshold_d;
    double threshold_a;
    double groundtruth_x;
    double groundtruth_y;
    double groundtruth_a;

    geometry_msgs::msg::TransformStamped last_tf_odom;
    sensor_msgs::msg::LaserScan real_scan;
    sensor_msgs::msg::LaserScan simul_sensor_specs;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particles{nullptr};
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_static_map{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gz_pose;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
	this->real_scan = *msg;
    }

    void gz_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
	this->groundtruth_x = msg->poses[0].position.x + 2.2508;
	this->groundtruth_y = msg->poses[0].position.y + 1.5;
	double w = msg->poses[0].orientation.w;
	double z = msg->poses[0].orientation.z;
	this->groundtruth_a = atan2(z, w)*2;
	if(this->groundtruth_a >   M_PI) this->groundtruth_a -= 2*M_PI;
	if(this->groundtruth_a <= -M_PI) this->groundtruth_a += 2*M_PI;
    }

    geometry_msgs::msg::TransformStamped get_odometry(){
	geometry_msgs::msg::TransformStamped t;
	try {
	    t = tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
	} catch (const tf2::TransformException & ex) {
	    RCLCPP_WARN(this->get_logger(), "Could not get odom transform: %s", ex.what());
	    return this->last_tf_odom;
	}
	return t;
    }

    bool displacement_greater_than_threshold(double& delta_x, double& delta_y, double& delta_a){
	geometry_msgs::msg::TransformStamped t = this->get_odometry();
	double delta_x_odom = t.transform.translation.x - this->last_tf_odom.transform.translation.x;
	double delta_y_odom = t.transform.translation.y - this->last_tf_odom.transform.translation.y;
	double current_a = atan2(t.transform.rotation.z, t.transform.rotation.w)*2;
	double last_a    = atan2(this->last_tf_odom.transform.rotation.z, this->last_tf_odom.transform.rotation.w)*2;
	delta_x =  delta_x_odom * cos(last_a) + delta_y_odom * sin(last_a);
	delta_y = -delta_x_odom * sin(last_a) + delta_y_odom * cos(last_a);
	delta_a = current_a - last_a;
	if(delta_a >   M_PI) delta_a -= 2*M_PI;
	if(delta_a <= -M_PI) delta_a += 2*M_PI;
	if(sqrt(delta_x*delta_x + delta_y*delta_y) > this->threshold_d || fabs(delta_a) > this->threshold_a){
	    this->last_tf_odom = t;
	    return true;
	}
	return false;
    }

    geometry_msgs::msg::PoseArray get_pose_array(std::vector<geometry_msgs::msg::Pose2D>& particles){
	geometry_msgs::msg::PoseArray poses;
	poses.header.frame_id = "map";
	poses.poses.resize(particles.size());
	for(size_t i=0; i < particles.size(); i++)
	{
	    poses.poses[i].position.x = particles[i].x;
	    poses.poses[i].position.y = particles[i].y;
	    poses.poses[i].orientation.z = sin(particles[i].theta/2);
	    poses.poses[i].orientation.w = cos(particles[i].theta/2);
	}
	return poses;
    }

    void get_estimated_pose(std::vector<geometry_msgs::msg::Pose2D>& particles, double& x, double& y, double& a){
	double mean_x = 0;
	double mean_y = 0;
	double mean_z = 0;
	double mean_w = 0;
	for(size_t i=0; i < particles.size(); i++){
	    mean_x += particles[i].x;
	    mean_y += particles[i].y;
	    mean_z += sin(particles[i].theta);
	    mean_w += cos(particles[i].theta);
	}
	mean_x /= particles.size();
	mean_y /= particles.size();
	mean_z /= particles.size();
	mean_w /= particles.size();
	x = mean_x;
	y = mean_y;
	a = atan2(mean_z, mean_w);
    }

    geometry_msgs::msg::TransformStamped get_odom_to_map(std::vector<geometry_msgs::msg::Pose2D>& particles){
	geometry_msgs::msg::TransformStamped t_map;
	geometry_msgs::msg::TransformStamped t_odom = this->get_odometry();
	double mean_x = 0;
	double mean_y = 0;
	double mean_z = 0;
	double mean_w = 0;
	for(size_t i=0; i < particles.size(); i++){
	    mean_x += particles[i].x;
	    mean_y += particles[i].y;
	    mean_z += sin(particles[i].theta/2);
	    mean_w += cos(particles[i].theta/2);
	}
	mean_x /= particles.size();
	mean_y /= particles.size();
	mean_z /= particles.size();
	mean_w /= particles.size();
	tf2::Transform map_to_base;
	map_to_base.setOrigin(tf2::Vector3(mean_x, mean_y, 0));
	map_to_base.setRotation(tf2::Quaternion(0,0,mean_z, mean_w));
	tf2::Transform odom_to_base;
	odom_to_base.setOrigin(tf2::Vector3(t_odom.transform.translation.x, t_odom.transform.translation.y, 0));
	odom_to_base.setRotation(tf2::Quaternion(0, 0, t_odom.transform.rotation.z, t_odom.transform.rotation.w));
	tf2::Transform map_to_odom = map_to_base*odom_to_base.inverse();
	
	t_map.transform.translation.x = map_to_odom.getOrigin().x();
	t_map.transform.translation.y = map_to_odom.getOrigin().y();
	t_map.transform.translation.z = 0;
	t_map.transform.rotation.x = 0;
	t_map.transform.rotation.y = 0;
	t_map.transform.rotation.z = map_to_odom.getRotation().z();
	t_map.transform.rotation.w = map_to_odom.getRotation().w();
	t_map.header.stamp = this->get_clock()->now();
	t_map.header.frame_id = "map";
	t_map.child_frame_id = "odom";
	return t_map;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto n = std::make_shared<ParticleFilterNode>();
  n->spin();
  rclcpp::shutdown();
  return 0;
}

