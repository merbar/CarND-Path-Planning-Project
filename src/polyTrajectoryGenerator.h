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
#include <random>

using namespace std;

class PolyTrajectoryGenerator {
public:
    PolyTrajectoryGenerator();
    ~PolyTrajectoryGenerator();
    
    vector<vector<double>> generate_trajectory(vector<double> const &start, double max_speed, double horizon, vector<vector<double>> const &sensor_fusion);
    void perturb_goal(vector<double> goal, vector<vector<double>> &goal_points);
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
    vector<double> jmt(double start, double goal, int t);
    
private:
    const double car_width = 1.5;
    const double car_length = 3.0;
    const double col_buf_width = 0.5 * car_width;
    const double col_buf_length = 1.5 * car_length;
    const int goal_perturb_samples = 10;
    double delta_s_maxspeed = 0.0;
    std::default_random_engine rand_generator;
};

#endif /* POLYTRAJECTORYGENERATOR_H */

