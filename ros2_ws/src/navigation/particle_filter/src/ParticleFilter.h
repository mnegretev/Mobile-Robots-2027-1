/*
 * MOBILE ROBOTS - UNAM, FI, 2027-1
 * LOCALIZATION BY PARTICLE FILTERS
 *
 * Instructions:
 * Write the code necessary to implement localization by particle filters.
 * Modify only the sections marked with the TODO comment. 
 */
#include "particle_filter/ray_tracer.h"
#define FULL_NAME "Martinez Cruz Lizeth"

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
	for (int i = 0; i < N; i++)
{
    particles[i].x = rnd.uniformReal(min_x, max_x);
    particles[i].y = rnd.uniformReal(min_y, max_y);
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
	for (auto& particle : particles)
{
    double angle = particle.theta;

    double new_x = particle.x
        + delta_x * cos(angle) - delta_y * sin(angle);

    double new_y = particle.y
        + delta_x * sin(angle) + delta_y * cos(angle);

    particle.x = rnd.gaussian(new_x, sigma2);
    particle.y = rnd.gaussian(new_y, sigma2);
    particle.theta = rnd.gaussian(angle + delta_t, sigma2);

    particle.theta = atan2(sin(particle.theta),
                           cos(particle.theta));
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
	double total = 0.0;

for (size_t i = 0; i < simulated_scans.size(); i++)
{
    double difference_sum = 0.0;
    size_t valid_readings = 0;

    for (size_t j = 0; j < simulated_scans[i].ranges.size(); j++)
    {
        size_t real_index = j * downsampling;

        if (real_index >= real_scan.ranges.size())
            break;

        double simulated = simulated_scans[i].ranges[j];
        double real = real_scan.ranges[real_index];

        if (std::isfinite(simulated) && std::isfinite(real))
        {
            difference_sum += std::abs(real - simulated);
            valid_readings++;
        }
    }

    if (valid_readings > 0)
    {
        double delta = difference_sum / valid_readings;
        similarities[i] = std::exp(-delta / sigma2);
    }
    else
    {
        similarities[i] = 0.0;
    }

    total += similarities[i];
}

// Normalizar para que las probabilidades sumen 1.
if (total > 0.0)
{
    for (auto& similarity : similarities)
        similarity /= total;
}
else if (!similarities.empty())
{
    for (auto& similarity : similarities)
        similarity = 1.0 / similarities.size();
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
	if (probabilities.empty())
    return -1;

double sample = rnd.uniformReal(0.0, 1.0);
double accumulated = 0.0;

for (size_t i = 0; i < probabilities.size(); i++)
{
    accumulated += probabilities[i];

    if (sample < accumulated)
        return static_cast<int>(i);
}

return static_cast<int>(probabilities.size() - 1);
	
	
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
	for (size_t i = 0; i < particles.size(); i++)
{
    int index = random_choice(probabilities);

    if (index < 0)
        return particles;

    resampled_particles[i].x =
        rnd.gaussian(particles[index].x, sigma2);

    resampled_particles[i].y =
        rnd.gaussian(particles[index].y, sigma2);

    double angle =
        rnd.gaussian(particles[index].theta, sigma2);

    resampled_particles[i].theta =
        atan2(sin(angle), cos(angle));
}
	/*
	 */
	return resampled_particles;
    }
    
};
