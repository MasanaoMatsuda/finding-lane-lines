#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <vector>
#include <iostream>
#include <random>
#include <iterator>
#include "helper_functions.h"

struct Particle {
	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};

class ParticleFilter {
	int num_particles;
	bool is_initialized;
	std::vector<double> weights;
public:
	std::vector<Particle> particles;

	ParticleFilter() : num_particles(0), is_initialized(false) {}
	
	~ParticleFilter() {}
	
	void init(double x, double y, double theta, double stddevs[]);
	
	void predict(double delta_t, double stddevs[], double velocity, double yaw_rate);
	
	void associateData(std::vector<Observation> predicted,
					   std::vector<Observation>& observations);
	
	void updateWeights(double sensor_range, double std_landmark[], 
					   const std::vector<Observation> &observations, 
					   const Map &map_landmarks);
	
	void resample();

	void setAssociations(Particle& particle, 
						 const std::vector<int>& associations,
						 const std::vector<double>& sense_x, 
						 const std::vector<double>& sense_y);
	
	std::string getAssociations(Particle best);

	std::string getSenseCoord(Particle best, std::string coordinate);

	const bool getIsInitialized() const { return is_initialized; };
};

#endif // !PARTICLE_FILTER_H_

