#include "particle_filter.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double stddev[])
{
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	std::normal_distribution<> dist_x(x, stddev[0]);
	std::normal_distribution<> dist_y(y, stddev[1]);
	std::normal_distribution<> dist_theta(theta, stddev[2]);

	vector<Particle> particles;
	for (int i = 0; i < num_particles; ++i)
	{
		Particle particle;
		particle.id = i;
		particle.x = dist_x(engine);
		particle.y = dist_y(engine);
		particle.theta = dist_theta(engine);

		particles.push_back(particle);
	}

	is_initialized = true;
}


void ParticleFilter::predict(double delta_t, double stddevs[], double velocity, double yaw_rate)
{
	std::cout << "Predict" << std::endl;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const vector<Observation> & observations, const Map & map_landmarks)
{
	std::cout << "UpdateWeights" << std::endl;
}

void ParticleFilter::resample()
{
	std::cout << "Resample" << std::endl;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	std::stringstream ss;
	copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);
	return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coordinate)
{
	vector<double> v;
	if (coordinate == "X")
		v = best.sense_x;
	else
		v = best.sense_y;

	std::stringstream ss;
	copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);
	return s;
}