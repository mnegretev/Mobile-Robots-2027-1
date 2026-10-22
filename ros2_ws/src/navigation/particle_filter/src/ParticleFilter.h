/*
- MOBILE ROBOTS - UNAM, FI, 2027-1
- LOCALIZATION BY PARTICLE FILTERS
 */
#include "particle_filter/ray_tracer.h"
#define FULL_NAME "Luis Yehosua Orihuela Castillo"

class ParticleFilter
{
public:
    ParticleFilter(){}

    static std::vector<geometry_msgs::msg::Pose2D> get_initial_distribution(
    int N, float min_x, float max_x, float min_y, float max_y, float min_a, float max_a)
    {
    random_numbers::RandomNumberGenerator rnd;
    std::vector<geometry_msgs::msg::Pose2D> particles(N);
    
    for(int i = 0; i < N; i++)
    {
        particles[i].x = rnd.uniformReal(min_x, max_x);
        particles[i].y = rnd.uniformReal(min_y, max_y);
        particles[i].theta = rnd.uniformReal(min_a, max_a);
    }

    return particles;
    }

    static void move_particles(std::vector<geometry_msgs::msg::Pose2D>& particles,
                   float delta_x, float delta_y, float delta_t, float sigma2)
    {
    random_numbers::RandomNumberGenerator rnd;
    double sigma = sqrt(sigma2);

    for(size_t i = 0; i < particles.size(); i++)
    {
        double theta = particles[i].theta;
        particles[i].x += delta_x * cos(theta) - delta_y * sin(theta) + rnd.gaussian(0, sigma);
        particles[i].y += delta_x * sin(theta) + delta_y * cos(theta) + rnd.gaussian(0, sigma);
        particles[i].theta += delta_t + rnd.gaussian(0, sigma);
    }
    }

    static std::vector<sensor_msgs::msg::LaserScan> simulate_particle_scans(
    std::vector<geometry_msgs::msg::Pose2D>& particles,
    nav_msgs::msg::OccupancyGrid& map,
    sensor_msgs::msg::LaserScan& sensor_specs)
    {
    std::vector<sensor_msgs::msg::LaserScan> simulated_scans(particles.size());
    for(size_t i=0; i < particles.size(); i++)
    {
        geometry_msgs::msg::Pose sensor_pose;
        sensor_pose.position.x    = particles[i].x;
        sensor_pose.position.y    = particles[i].y;
        sensor_pose.orientation.w = cos(particles[i].theta/2);
        sensor_pose.orientation.z = sin(particles[i].theta/2);
        
        simulated_scans[i] = ray_tracer::simulateRangeScan(map, sensor_pose, sensor_specs);
        
    }
    return simulated_scans;
    }

    static std::vector<double> get_particle_similarities(
    std::vector<sensor_msgs::msg::LaserScan>& simulated_scans,
    sensor_msgs::msg::LaserScan& real_scan,
    int downsampling, float sigma2)
    {
    std::vector<double> similarities;
    similarities.resize(simulated_scans.size());
    
    for(size_t i=0; i < simulated_scans.size(); i++)
    {
        double mean_diff = 0;
        for(size_t j=0; j < simulated_scans[i].ranges.size(); j++)
            if(real_scan.ranges[j*downsampling] < real_scan.range_max && simulated_scans[i].ranges[j] < real_scan.range_max)
                mean_diff += fabs(simulated_scans[i].ranges[j] - real_scan.ranges[j*downsampling]);
            else
                mean_diff += real_scan.range_max;

        mean_diff /= simulated_scans[i].ranges.size();
        similarities[i] = exp(-mean_diff/sigma2);
    }

    double sum = 0;
    for(size_t i=0; i<similarities.size(); i++)
        sum += similarities[i];

    for(size_t i=0; i<similarities.size(); i++)
        similarities[i] /= sum;
    
    return similarities;
    }
    
    static int random_choice(std::vector<double>& probabilities)
    {
    random_numbers::RandomNumberGenerator rnd;
    float beta = rnd.uniformReal(0, 1);
    for(size_t i=0; i < probabilities.size(); i++)
        if(beta < probabilities[i])
            return i;
        else
            beta -= probabilities[i];
    
    return -1;
    }

    static std::vector<geometry_msgs::msg::Pose2D> resample_particles(
    std::vector<geometry_msgs::msg::Pose2D>& particles, std::vector<double>& probabilities, float sigma2)
    {

    random_numbers::RandomNumberGenerator rnd;
    std::vector<geometry_msgs::msg::Pose2D> resampled_particles(particles.size());
    double sigma = sqrt(sigma2);            // rnd.gaussian expects a standard deviation
    for(size_t i=0; i < particles.size(); i++)
    {
        int idx = random_choice(probabilities);
        resampled_particles[i]          = particles[idx];
        resampled_particles[i].x       += rnd.gaussian(0, sigma);
        resampled_particles[i].y       += rnd.gaussian(0, sigma);
        resampled_particles[i].theta   += rnd.gaussian(0, sigma);
    }
    return resampled_particles;
    }
    
};