#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include "map.h"
#include "observation.h"
#include "control.h"
#include "ground_truth.h"

#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif // !M_PI

inline double euclidean_distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double* getError(double gt_x, double gt_y, double gt_theta,
	double pf_x, double pf_y, double pf_theta)
{
	static double error[3];
	error[0] = fabs(pf_x - gt_x);
	error[1] = fabs(pf_y - gt_y);
	error[2] = fabs(pf_theta - gt_theta);
	error[2] = fmod(error[2], 2.0 * M_PI);
	if (error[2] > M_PI)
		error[2] = 2.0 * M_PI - error[2];
	return error;
}

inline bool read_map_data(std::string filename, Map& map)
{
	std::ifstream in_file_map(filename.c_str(), std::ifstream::in);
	if (!in_file_map) {
		return false;
	}

	Map::landmark landmark;
	std::string line_map;
	while (getline(in_file_map, line_map))
	{
		std::istringstream map_iss(line_map);
		int landmark_id;
		float landmark_x, landmark_y;

		map_iss >> landmark_id;
		map_iss >> landmark_x;
		map_iss >> landmark_y;

		landmark.id = landmark_id;
		landmark.x = landmark_x;
		landmark.y = landmark_y;

		map.landmarks.push_back(landmark);
	}

	return true;
}

inline bool read_control_data(std::string filename,
	std::vector<Control>& control_meas)
{
	std::ifstream in_file_ctrl(filename.c_str(), std::ifstream::in);
	if (!in_file_ctrl)
		return false;

	std::string line_ctrl;
	while (getline(in_file_ctrl, line_ctrl))
	{
		std::istringstream iss_ctrl(line_ctrl);

		double velocity, yawrate;

		Control meas;
		iss_ctrl >> velocity;
		iss_ctrl >> yawrate;
		meas.velocity = velocity;
		meas.yaw_rate = yawrate;

		control_meas.push_back(meas);
	}
	return true;
}

inline bool read_ground_truth_data(std::string filename, std::vector<GroundTruth>& ground_truthes)
{
	std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);
	if (!in_file_pos)
		return false;

	std::string line_pos;
	while (getline(in_file_pos, line_pos))
	{
		std::istringstream iss_pos(line_pos);
		
		double x, y, azimuth;

		GroundTruth gt;
		iss_pos >> x;
		iss_pos >> y;
		iss_pos >> azimuth;
		gt.x = x;
		gt.y = y;
		gt.theta = azimuth;

		ground_truthes.push_back(gt);
	}
	return true;
}

inline bool read_landmark_data(std::string filename, std::vector<Observation>& observations)
{
	std::ifstream in_file_obs(filename.c_str(), std::ifstream::in);
	if (!in_file_obs) {
		return false;
	}

	std::string line_obs;
	while (getline(in_file_obs, line_obs))
	{
		std::istringstream iss_obs(line_obs);
		double local_x, local_y;
		iss_obs >> local_x;
		iss_obs >> local_y;

		Observation meas;
		meas.x = local_x;
		meas.y = local_y;

		observations.push_back(meas);
	}
	return true;
}

#endif // !HELPER_FUNCTIONS_H_


