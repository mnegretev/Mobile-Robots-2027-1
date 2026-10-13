/*
 * MOBILE ROBOTS - UNAM, FI, 2027-1
 * LOCALIZATION BY PARTICLE FILTERS
 *
 * Instructions:
 * Write the code necessary to implement localization by particle filters.
 * Modify only the sections marked with the TODO comment. 
 */
#include "particle_filter/ray_tracer.h"
#define FULL_NAME "FULL NAME"

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
	for(int i=0; i < N; i++)
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
	// NOTE: rnd.gaussian(mean, stddev) receives a standard deviation, so sqrt(sigma2) is used
	// to make sigma2 behave as a variance.
	double sigma = sqrt(sigma2);
	for(size_t i=0; i < particles.size(); i++)
	{
	    double c = cos(particles[i].theta);
	    double s = sin(particles[i].theta);
	    // Rotate the displacement from the particle's frame to the world frame, then add noise
	    particles[i].x     += delta_x*c - delta_y*s + rnd.gaussian(0, sigma);
	    particles[i].y     += delta_x*s + delta_y*c + rnd.gaussian(0, sigma);
	    particles[i].theta += delta_t + rnd.gaussian(0, sigma);
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
	size_t n = simulated_scans.size();
	if(n == 0)
	    return similarities;
	if(downsampling < 1)
	    downsampling = 1;

	// Step 1. Mean squared error between each simulated scan and the real scan.
	// An error of -1 marks a particle with no comparable (finite) readings.
	std::vector<double> errors(n, -1.0);
	double min_error = -1.0;
	for(size_t i=0; i < n; i++)
	{
	    double sum = 0;
	    int count = 0;
	    for(size_t j=0; j < simulated_scans[i].ranges.size(); j++)
	    {
		size_t k = j*downsampling;           // matching reading in the real (not downsampled) scan
		if(k >= real_scan.ranges.size())
		    break;
		double sim  = simulated_scans[i].ranges[j];
		double real = real_scan.ranges[k];
		if(!std::isfinite(sim) || !std::isfinite(real))
		    continue;
		sum += (sim - real)*(sim - real);
		count++;
	    }
	    if(count > 0)
	    {
		errors[i] = sum/count;
		if(min_error < 0 || errors[i] < min_error)
		    min_error = errors[i];
	    }
	}

	// Nothing comparable at all: fall back to a uniform distribution
	if(min_error < 0)
	{
	    for(size_t i=0; i < n; i++)
		similarities[i] = 1.0/n;
	    return similarities;
	}

	// Step 2. Gaussian likelihood exp(-error/sigma2).
	// The minimum error is subtracted first so the best particle always gets exp(0)=1 and
	// the exponentials do not underflow to zero for every particle.
	double total = 0;
	for(size_t i=0; i < n; i++)
	{
	    similarities[i] = (errors[i] < 0) ? 0.0 : exp(-(errors[i] - min_error)/sigma2);
	    total += similarities[i];
	}

	// Step 3. Normalize so that the sum is 1.0
	for(size_t i=0; i < n; i++)
	    similarities[i] /= total;
	
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
	// Roulette wheel: draw r in [0,1) and return the first index where the cumulative sum reaches r
	if(probabilities.empty())
	    return -1;
	
	double r = rnd.uniformReal(0.0, 1.0);
	double cumulative = 0;
	int last_valid = -1;                         // last index with non-zero probability
	for(size_t i=0; i < probabilities.size(); i++)
	{
	    if(probabilities[i] <= 0)
		continue;
	    last_valid = i;
	    cumulative += probabilities[i];
	    if(r <= cumulative)
		return i;
	}
	// Rounding errors can leave the cumulative sum slightly below 1.0
	return last_valid >= 0 ? last_valid : (int)(probabilities.size() - 1);
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
	double sigma = sqrt(sigma2);                 // rnd.gaussian expects a standard deviation
	for(size_t i=0; i < particles.size(); i++)
	{
	    int idx = random_choice(probabilities);
	    if(idx < 0)
		idx = i;                             // safety net: keep the particle if nothing could be chosen
	    resampled_particles[i]        = particles[idx];
	    resampled_particles[i].x     += rnd.gaussian(0, sigma);
	    resampled_particles[i].y     += rnd.gaussian(0, sigma);
	    resampled_particles[i].theta += rnd.gaussian(0, sigma);
	}
	/*
	 */
	return resampled_particles;
    }
    
};
