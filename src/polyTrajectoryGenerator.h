/* 
 * File:   polyTrajectoryGenerator.h
 * Author: merbar
 *
 * Created on July 19, 2017, 10:51 PM
 */

#ifndef POLYTRAJECTORYGENERATOR_H
#define POLYTRAJECTORYGENERATOR_H

#include <iostream>
#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class PolyTrajectoryGenerator {
public:
    PolyTrajectoryGenerator();
    ~PolyTrajectoryGenerator();
    
    vector<double> generate_trajectory(double start_s, double start_d, double max_speed, double horizon, vector<vector<double>> const &sensor_fusion);
    vector<double> perturb_goal(double s, double d);
    float calculate_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float lane_depart_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float collision_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float buffer_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float exceeds_speed_limit_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float max_accel_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float total_accel_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float max_jerk_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float total_jerk_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    float efficiency_cost(vector<double> const &traj, vector<vector<double>> const &sensor_fusion);
    vector<double> jmt(vector<double> const &start, vector<double> const &goal);

private:

};

#endif /* POLYTRAJECTORYGENERATOR_H */

