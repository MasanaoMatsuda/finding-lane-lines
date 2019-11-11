#include "particle_filter.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double stddev[])
{
	std::random_device seed_gen;
	std::mt19937 mt(seed_gen());

	std::normal_distribution<> dist_x(x, stddev[0]);
	std::normal_distribution<> dist_y(y, stddev[1]);
	std::normal_distribution<> dist_theta(theta, stddev[2]);

	for (int i = 0; i < num_particles; ++i)
	{
		Particle particle;
		particle.id = i;
		particle.x = dist_x(mt);
		particle.y = dist_y(mt);
		particle.theta = dist_theta(mt);

		particles.push_back(particle);
		weights.push_back(1);
	}

	is_initialized = true;
}


void ParticleFilter::predict(double delta_t, double stddevs[], double velocity, double yaw_rate)
{
	std::random_device seed_gen;
	std::mt19937 mt(seed_gen());

	std::cout << "Predict" << std::endl;
	for (int i = 0; i < num_particles; ++i)
	{
		particles[i].theta += yaw_rate * delta_t;
		particles[i].x += velocity * delta_t * cos(particles[i].theta);
		particles[i].y += velocity * delta_t * sin(particles[i].theta);

		std::normal_distribution<> dist_x(particles[i].x, stddevs[0]);
		std::normal_distribution<> dist_y(particles[i].y, stddevs[1]);
		std::normal_distribution<> dist_t(particles[i].theta, stddevs[2]);
		particles[i].x = dist_x(mt);
		particles[i].y = dist_y(mt);
		particles[i].theta = dist_t(mt);

		/*
		if (abs(yaw_rate) < 0.0001)
		{
			particles[i].x += (delta_t * velocity * (cos(particles[i].theta)));
			particles[i].y += (delta_t * velocity * (sin(particles[i].theta)));
		}
		else
		{
			particles[i].theta += yaw_rate * delta_t;
			particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
		}
		std::normal_distribution<double> dist_x(particles[i].x, stddevs[0]);
		std::normal_distribution<double> dist_y(particles[i].y, stddevs[1]);
		std::normal_distribution<double> dist_t(particles[i].theta, stddevs[2]);
		particles[i].x = dist_x(mt);
		particles[i].y = dist_y(mt);
		particles[i].theta = dist_t(mt);
		*/
	}
}

void ParticleFilter::associateData(vector<Observation> predicted, vector<Observation>& observations)
{
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const vector<Observation> & observations, const Map & map)
{
	std::cout << "UpdateWeights" << std::endl;

	double new_weight;
	for (int i = 0; i < num_particles; ++i)
	{
		new_weight = 1.0;
		for (int j = 0; j < observations.size(); ++j)
		{
			Observation transformed_obs;
			transformCoordinate(transformed_obs, observations[j], particles[i]);

			Map::landmark landmark = GetNearestNeighborLandmark(map, transformed_obs);

			new_weight *= multivariate_prob(std_landmark[0], std_landmark[1],
				transformed_obs.x, transformed_obs.y, landmark.x, landmark.y);
		}
		particles[i].weight = new_weight;
		weights[i] = new_weight;
	}
}

void ParticleFilter::resample()
{
	vector<Particle> new_particles;

	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());
	std::uniform_real_distribution<> dist(0.0, 1.0);

	double beta = 0.0;
	size_t max_idx = std::distance(weights.begin(), std::max_element(weights.begin(), weights.end()));
	int idx = (int)(dist(engine) * num_particles);
	for (int i = 0; i < num_particles; ++i)
	{
		beta += dist(engine) * 2 * weights[max_idx];
		while (weights[idx] < beta)
		{
			beta -= weights[idx];
			idx = (idx + 1) % num_particles;
		}
		new_particles.push_back(particles[idx]);
	}
	particles = new_particles;
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

void ParticleFilter::transformCoordinate(Observation& to_map_obs, const Observation& from_local_obs, const Particle& particle)
{
	to_map_obs.x = particle.x + (cos(particle.theta) * from_local_obs.x) - (sin(particle.theta) * from_local_obs.y);
	to_map_obs.y = particle.y + (sin(particle.theta) * from_local_obs.x) + (cos(particle.theta) * from_local_obs.y);
}

Map::landmark ParticleFilter::GetNearestNeighborLandmark(const Map& map, const Observation &observation)
{
	double nearest = DBL_MAX;
	int nearest_idx;
	for (int i = 0; i < map.landmarks.size(); ++i)
	{
		double distance = euclidean_distance(map.landmarks[i].x, map.landmarks[i].y, observation.x, observation.y);
		if (distance < nearest)
		{
			nearest = distance;
			nearest_idx = i;
		}
	}
	return map.landmarks[nearest_idx];
}