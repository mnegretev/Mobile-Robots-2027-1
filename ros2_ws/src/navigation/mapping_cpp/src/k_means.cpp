/*
 * MOBILE ROBOTS - FI-UNAM, 2027-1
 * CLUSTERING BY K-MEANS
 * Instructions:
 * Write the code necessary to implement the K-means algorithm to cluster a map
 * given an occupancy grid and a number of centroids K
 * Modify only the sections with the TODO comment
 */

#include <cstdio>
#include <limits>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "random_numbers/random_numbers.h"
#include "visualization_msgs/msg/marker.hpp"

#define FULL_NAME "LEONARDO ALEJANDRO GARCIA LERMA"

class KMeansNode : public rclcpp::Node
{
public:
    KMeansNode():Node("k_means"){
	RCLCPP_INFO(this->get_logger(), "INITIALIZING K-MEANS NODE - %s", FULL_NAME);
	K = this->declare_parameter<int>("K", 3);
	tol = this->declare_parameter<double>("tol", 0.2);
	clt_inflated_map = this->create_client<nav_msgs::srv::GetMap>("/get_inflated_map");
	pub_marker = this->create_publisher<visualization_msgs::msg::Marker>("/mapping/k_means_centroids", 1);
    }

    std::vector<Eigen::Vector2d> get_cartesian_free_points(nav_msgs::msg::OccupancyGrid& map){
	std::vector<Eigen::Vector2d> P;
	for(size_t i=0; i < map.data.size(); i++){
	    if(map.data[i] != 0)
		continue;
	    int row = i / map.info.width;
	    int col = i % map.info.width;
	    double x = col * map.info.resolution + map.info.origin.position.x;
	    double y = row * map.info.resolution + map.info.origin.position.y;
	    Eigen::Vector2d p(x,y);
	    P.push_back(p);
	}
	RCLCPP_INFO(this->get_logger(), "Got map with %lu points in free space", P.size());
	return P;
    }

    std::vector<Eigen::Vector2d> generate_random_centroids(int K, double min_x, double min_y, double max_x, double max_y,
							   nav_msgs::msg::OccupancyGrid& map){
	std::vector<Eigen::Vector2d> C;
	random_numbers::RandomNumberGenerator rnd;
	for(int i=0; i < K; i++){
	    bool in_free_space = false;
	    double x,y;
	    while(!in_free_space){
		x = rnd.uniformReal(min_x, max_x);
		y = rnd.uniformReal(min_y, max_y);
		int row = (int)((y - map.info.origin.position.y)/map.info.resolution);
		int col = (int)((x - map.info.origin.position.x)/map.info.resolution);
		in_free_space = map.data[row*map.info.width + col] == 0;
	    }
	    Eigen::Vector2d c(x,y);
	    C.push_back(c);
	}
	RCLCPP_INFO(this->get_logger(), "Generated %lu random centroids", C.size());
	return C;
    }

    std::vector<Eigen::Vector2d> recalculate_centroids(std::vector<Eigen::Vector2d>& centroids,
						       std::vector<Eigen::Vector2d>& points){
	RCLCPP_INFO(this->get_logger(), "Recalculating centroids");
	std::vector<Eigen::Vector2d> new_centroids(centroids.size());	    
	std::vector<int> counters(centroids.size());
	/*
	 * TODO:
	 * Implement the steps to recalculate centroids.
	 * Use as reference the python version of this algorithm.
	 * Use the declared variables and the Eigen library
	 */
	while (max_distance > tol) {
    RCLCPP_INFO(this->get_logger(), "Recalculating centroids");

    auto new_centroids = recalculate_centroids(centroids, P);

    // Calcular la distancia máxima entre los centroides viejos y nuevos
    max_distance = 0.0;
    for (size_t i = 0; i < centroids.size(); ++i) {
        double dist = std::hypot(centroids[i].x - new_centroids[i].x,
                                 centroids[i].y - new_centroids[i].y);
        if (dist > max_distance) {
            max_distance = dist;
        }
    }

    centroids = new_centroids;
    iterations += 1;
    pub_centroids_->publish(get_centroids_marker(centroids));
}
	/*
	 * END OF TODO
	 */
	return new_centroids;
    }

    visualization_msgs::msg::Marker get_centroids_marker(std::vector<Eigen::Vector2d>& centroids){
	visualization_msgs::msg::Marker mrk;
	mrk.header.frame_id = "map";
        mrk.header.stamp = this->get_clock()->now();
        mrk.ns = "mapping";
	mrk.lifetime.sec=10000;
        mrk.id = 0;
        mrk.type   = visualization_msgs::msg::Marker::SPHERE_LIST;
        mrk.action = visualization_msgs::msg::Marker::ADD;
	mrk.scale.x = 0.2;
	mrk.scale.y = 0.2;
	mrk.scale.z = 0.2;
	mrk.color.r = 1.0;
	mrk.color.a = 1.0;
	for(size_t i=0; i < centroids.size(); i++){
	    geometry_msgs::msg::Point p;
	    p.x = centroids[i][0];
	    p.y = centroids[i][1];
	    mrk.points.push_back(p);
	}
	return mrk;
    }

    void spin()
    {
	/*
	 * Waiting for inflated map service to be available
	 */
	RCLCPP_INFO(this->get_logger(), "Waiting for inflated map");
	while(!this->clt_inflated_map->wait_for_service(std::chrono::seconds(1))){
	    if(!rclcpp::ok()){
		RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the inflated map service. Exiting.");
		return;
	    }
	    RCLCPP_INFO(this->get_logger(), "Inflated map service not available, waiting again...");
	}
	/*
	 * Getting inflated map
	 */
	nav_msgs::msg::OccupancyGrid inflated_map;
	RCLCPP_INFO(this->get_logger(), "Inflated map service is now available. Getting inflated map.");
	nav_msgs::srv::GetMap::Request::SharedPtr req = std::make_shared<nav_msgs::srv::GetMap::Request>();
	rclcpp::Client<nav_msgs::srv::GetMap>::FutureAndRequestId result = this->clt_inflated_map->async_send_request(req);
	if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS){
	    inflated_map = result.get()->map;
	    RCLCPP_INFO(this->get_logger(), "Got map with size %d x %d", inflated_map.info.width, inflated_map.info.height);
	}else{
	    RCLCPP_INFO(this->get_logger(), "Cannot get inflated map. Aborting node. ");
	    return;
	}
	
	std::vector<Eigen::Vector2d> P = get_cartesian_free_points(inflated_map);
	std::vector<Eigen::Vector2d> centroids = generate_random_centroids(this->K, -5, -5, 10, 5,  inflated_map);
	visualization_msgs::msg::Marker mrk = get_centroids_marker(centroids);
	pub_marker->publish(mrk);

	double max_dist = std::numeric_limits<double>::max();
	while(rclcpp::ok() && max_dist > this->tol){
	    pub_marker->publish(mrk);
	    rclcpp::spin_some(this->get_node_base_interface());
	    rclcpp::sleep_for(std::chrono::milliseconds(100));
	    /*
	     * TODO:
	     * Recalculate centroids
	     * Get the maximum distance between each centroids and is corresponding new centroid
	     * Use as reference the python version of this algorithm.
	     * Use the declared variables and the Eigen library
	     */
	    for (const auto& p : P) {
    int best_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    // Encontrar el centroide más cercano
    for (size_t c = 0; c < centroids.size(); ++c) {
        double dist = std::hypot(p.x - centroids[c].x, p.y - centroids[c].y);
        if (dist < min_dist) {
            min_dist = dist;
            best_idx = c;
        }
    }

    // Sumar las coordenadas del punto al cluster correspondiente
    clusters[best_idx].x += p.x;
    clusters[best_idx].y += p.y;
    counters[best_idx] += 1;
}

// Calcular el promedio para obtener los nuevos centroides
for (size_t i = 0; i < centroids.size(); ++i) {
    if (counters[i] > 0) {
        new_centroids[i].x = clusters[i].x / counters[i];
        new_centroids[i].y = clusters[i].y / counters[i];
    }
}
	    /*
	     * END OF TODO
	     */
	    RCLCPP_INFO(this->get_logger(), "Max change in centroids: %lf", max_dist);
	    mrk = get_centroids_marker(centroids);
	}
    }

private:
    int K;
    double tol;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_inflated_map{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker{nullptr};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto n = std::make_shared<KMeansNode>();
  n->spin();
  rclcpp::shutdown();
  return 0;
}

