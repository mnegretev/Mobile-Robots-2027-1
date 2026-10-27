/*
- MOBILE ROBOTS - UNAM, FI, 2027-1
- LOCALIZATION BY PARTICLE FILTERS
 *
- Instructions:
- Write the code necessary to implement localization by particle filters.
- Modify only the sections marked with the TODO comment. 
 */
#include "particle_filter/ray_tracer.h"
#include <cmath>

#define FULL_NAME "Leonardo Alejandro Garcia Lerma"

class ParticleFilter
{
public:
    ParticleFilter(){}

    static std::vector<geometry_msgs::msg::Pose2D> get_initial_distribution(
    int N, float min_x, float max_x, float min_y, float max_y, float min_a, float max_a)
    {
        random_numbers::RandomNumberGenerator rnd;
        std::vector<geometry_msgs::msg::Pose2D> particles(N);
        
        // Generación de N partículas con distribución uniforme[cite: 1]
        for(int i = 0; i < N; i++) {
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
        
        // Actualización de posición aplicando rotación y ruido Gaussiano[cite: 2]
        for(size_t i = 0; i < particles.size(); i++) {
            float nx = rnd.gaussian(0.0, sigma2);
            float ny = rnd.gaussian(0.0, sigma2);
            float nt = rnd.gaussian(0.0, sigma2);
            
            float x_new = particles[i].x + delta_x * cos(particles[i].theta) - delta_y * sin(particles[i].theta) + nx;
            float y_new = particles[i].y + delta_x * sin(particles[i].theta) + delta_y * cos(particles[i].theta) + ny;
            float t_new = particles[i].theta + delta_t + nt;
            
            particles[i].x = x_new;
            particles[i].y = y_new;
            particles[i].theta = t_new;
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
        double sum_similarities = 0.0;
        
        // Cálculo de similitud manejando downsampling y valores finitos[cite: 1, 2]
        for(size_t i = 0; i < simulated_scans.size(); i++) {
            double delta_sum = 0.0;
            int valid_readings = 0;
            
            for(size_t j = 0; j < simulated_scans[i].ranges.size(); j++) {
                float sim_r = simulated_scans[i].ranges[j];
                float real_r = real_scan.ranges[j * downsampling];
                
                if(std::isfinite(sim_r) && std::isfinite(real_r)) {
                    delta_sum += std::abs(real_r - sim_r);
                    valid_readings++;
                }
            }
            
            double delta = (valid_readings > 0) ? (delta_sum / valid_readings) : 0.0;
            similarities[i] = exp(-delta / sigma2);
            sum_similarities += similarities[i];
        }
        
        // Normalización para crear distribución de probabilidad discreta[cite: 1]
        if (sum_similarities > 0.0) {
            for(size_t i = 0; i < similarities.size(); i++) {
                similarities[i] /= sum_similarities;
            }
        }
        
        return similarities;
    }
    
    static int random_choice(std::vector<double>& probabilities)
    {
        random_numbers::RandomNumberGenerator rnd;
        
        // Selección aleatoria basada en probabilidad discreta[cite: 1]
        double r = rnd.uniformReal(0.0, 1.0);
        double cumulative = 0.0;
        
        for(size_t i = 0; i < probabilities.size(); i++) {
            cumulative += probabilities[i];
            if(r <= cumulative) {
                return i;
            }
        }
        return probabilities.size() - 1; 
    }

    static std::vector<geometry_msgs::msg::Pose2D> resample_particles(
    std::vector<geometry_msgs::msg::Pose2D>& particles, std::vector<double>& probabilities, float sigma2)
    {
        random_numbers::RandomNumberGenerator rnd;
        std::vector<geometry_msgs::msg::Pose2D> resampled_particles(particles.size());
        
        // Remuestreo con reemplazo y adición de ruido Gaussiano[cite: 1]
        for(size_t i = 0; i < particles.size(); i++) {
            int idx = random_choice(probabilities);
            geometry_msgs::msg::Pose2D p = particles[idx];
            
            p.x += rnd.gaussian(0.0, sigma2);
            p.y += rnd.gaussian(0.0, sigma2);
            p.theta += rnd.gaussian(0.0, sigma2);
            
            resampled_particles[i] = p;
        }
        
        return resampled_particles;
    }
    
};