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
#include "Eigen-3.3/Eigen/LU"
#include <random>
#include "Polynomial.h"

using namespace std;

class PolyTrajectoryGenerator {
public:
    PolyTrajectoryGenerator();
    ~PolyTrajectoryGenerator();
    
    vector<vector<double>> generate_trajectory(vector<double> const &start, double max_speed, double horizon, vector<vector<double>> const &sensor_fusion);
    void perturb_goal(vector<double> goal, vector<vector<double>> &goal_points);
    vector<vector<double>> closest_vehicle_in_lane(vector<double> const &start, vector<vector<double>> const &sensor_fusion);
    float calculate_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    float exceeds_speed_limit_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    float exceeds_accel_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    float exceeds_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    
    float lane_depart_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    float collision_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    float buffer_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    float total_accel_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    float total_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);
    float efficiency_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion);

    Polynomial jmt(vector<double> const &start, vector<double> const &goal, int t);
    
private:
    const double _car_width = 1.5;
    const double _car_length = 3.0;
    const double _col_buf_width = 0.5 * _car_width;
    const double _col_buf_length = 1.5 * _car_length;
    const int _goal_perturb_samples = 30;
    int _horizon = 0;
    const double _hard_max_vel_per_timestep = 0.00894 * 50.0; // 50 mp/h
    const double _hard_max_acc_per_timestep = 10.0 / 50.0; // 10 m/s
    const double _hard_max_jerk_per_timestep = 10.0 / 50.0; // 10 m/s
    double _max_dist_per_timestep = 0.0;
    double _delta_s_maxspeed = 0.0;
    std::default_random_engine rand_generator;
};

#endif /* POLYTRAJECTORYGENERATOR_H */

