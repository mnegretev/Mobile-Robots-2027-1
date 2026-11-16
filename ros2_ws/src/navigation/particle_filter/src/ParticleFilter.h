/*
 * MOBILE ROBOTS - UNAM, FI, 2027-1
 * LOCALIZATION BY PARTICLE FILTERS
 *
 * Instructions:
 * Write the code necessary to implement localization by particle filters.
 * Modify only the sections marked with the TODO comment. 
 */
#include "particle_filter/ray_tracer.h"
#define FULL_NAME "PEREZ CORTES NATHAN"

class ParticleFilter
{
public:
    ParticleFilter(){}

    static std::vector<geometry_msgs::msg::Pose2D> get_initial_distribution(
    int N, float min_x, float max_x, float min_y, float max_y, float min_a, float max_a)
    {
	random_numbers::RandomNumberGenerator rnd;
	std::vector<geometry_msgs::msg::Pose2D> particles(N);
	/*
	 * TODO:
	 * Generate a set of N particles (each particle represented by a Pose2D message)
	 * with positions uniformly distributed within bounding box given by min_x, ..., max_a.
	 * To generate uniformly distributed random numbers, you can use the funcion rnd.uniformReal(min, max)
	 */
	for(int i = 0; i < N; i++)
	{
	    particles[i].x     = rnd.uniformReal(min_x, max_x);
	    particles[i].y     = rnd.uniformReal(min_y, max_y);
	    particles[i].theta = rnd.uniformReal(min_a, max_a);
	}
	/*
	 */
	return particles;
    }

    static void move_particles(std::vector<geometry_msgs::msg::Pose2D>& particles,
			       float delta_x, float delta_y, float delta_t, float sigma2)
    {
	random_numbers::RandomNumberGenerator rnd;
	/*
	 * TODO:
	 * Move each particle a displacement given by delta_x, delta_y and delta_t.
	 * Displacement is given w.r.t. particles's frame, i.e., to calculate the new position for
	 * each particle you need to rotate delta_x and delta_y, on Z axis, an angle theta_i, where theta_i
	 * is the orientation of the i-th particle.
	 * Add gaussian noise to each new position. Use sigma2 as variance.
	 * You can use the function rnd.gaussian(mean, variance)
	 */
	for(size_t i = 0; i < particles.size(); i++)
	{
	    double c = cos(particles[i].theta);
	    double s = sin(particles[i].theta);
	    particles[i].x     += delta_x*c - delta_y*s + rnd.gaussian(0, sigma2);
	    particles[i].y     += delta_x*s + delta_y*c + rnd.gaussian(0, sigma2);
	    particles[i].theta += delta_t               + rnd.gaussian(0, sigma2);
	}
    }

    static std::vector<sensor_msgs::msg::LaserScan> simulate_particle_scans(
	std::vector<geometry_msgs::msg::Pose2D>& particles,
	nav_msgs::msg::OccupancyGrid& map,
	sensor_msgs::msg::LaserScan& sensor_specs)
    {
	/*
	 * TODO:
	 * Review the code to simulate a laser scan for each particle given the set of particles and a static map. 
	 */
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
	/*
	 * TODO:
	 * For each particle, calculate the similarity between its simulated scan and the real scan.
	 * Normalize all similarities (the sum of all values must always be 1.0)
	 * Store results in 'similarities'.
	 * IMPORTANT NOTE 1. The real sensor scans are DOWNSAMPLED. That is, only 1 out of 'downsampling' scans is considered, i.e.,
	 * For example, if downsampling=10, then, if real sensor has 500 ranges, simulated scans will only have 50 ranges
	 * When comparing readings, for each reading in the simulated scan, you should skip 'downsampling' readings
	 * in the real sensor.
	 * IMPORTANT NOTE 2. Both, simulated an real scans, can have infinite distances. Thus, when comparing readings,
	 * ensure both simulated and real ranges are finite values. 
	 */
		double sum = 0;
	for(size_t i = 0; i < simulated_scans.size(); i++)
	{
	    double delta = 0;
	    int valid = 0;
	    for(size_t j = 0; j < simulated_scans[i].ranges.size(); j++)
	    {
		size_t k = j * downsampling;
		if(k >= real_scan.ranges.size())
		    break;
		float ds = simulated_scans[i].ranges[j];
		float dr = real_scan.ranges[k];
		if(std::isfinite(ds) && std::isfinite(dr))
		{
		    delta += fabs(dr - ds);
		    valid++;
		}
	    }
	    if(valid > 0)
		similarities[i] = exp(-(delta / valid) / sigma2);
	    else
		similarities[i] = 0;
	    sum += similarities[i];
	}
	if(sum > 0)
	{
	    for(size_t i = 0; i < similarities.size(); i++)
		similarities[i] /= sum;
	}
	else  
	{
	    for(size_t i = 0; i < similarities.size(); i++)
		similarities[i] = 1.0 / similarities.size();
	}
	return similarities;
    }
    
    static int random_choice(std::vector<double>& probabilities)
    {
	random_numbers::RandomNumberGenerator rnd;
	/*
	 * TODO:
	 *
	 * Write an algorithm to choice an integer in the range [0, N-1], with N, the length of 'probabilities'.
	 * Probability of picking an integer 'i' is given by the corresponding probabilities[i] value.
	 * Return the chosen integer. 
	 */
	double r = rnd.uniformReal(0.0, 1.0);
	double cumulative = 0;
	for(size_t i = 0; i < probabilities.size(); i++)
	{
	    cumulative += probabilities[i];
	    if(r <= cumulative)
		return i;
	}
	return probabilities.size() - 1;

    }

    static std::vector<geometry_msgs::msg::Pose2D> resample_particles(
	std::vector<geometry_msgs::msg::Pose2D>& particles, std::vector<double>& probabilities, float sigma2)
    {

	random_numbers::RandomNumberGenerator rnd;
	std::vector<geometry_msgs::msg::Pose2D> resampled_particles(particles.size());
	/*
	 * TODO:
	 * Sample, with replacement, N particles from the set 'particles'.
	 * The probability of the i-th particle to be resampled is given by probabilities[i].
	 * Use the random_choice function to pick a particle with the correct probability.
	 * Add gaussian noise to each sampled particle (add noise to x,y and theta). Use sigma2 as noise variance.
	 */
	for(size_t i = 0; i < particles.size(); i++)
	{
	    int idx = random_choice(probabilities);
	    resampled_particles[i].x     = particles[idx].x     + rnd.gaussian(0, sigma2);
	    resampled_particles[i].y     = particles[idx].y     + rnd.gaussian(0, sigma2);
	    resampled_particles[i].theta = particles[idx].theta + rnd.gaussian(0, sigma2);
	}
	/*
	 */
	return resampled_particles;
    }
    
};
