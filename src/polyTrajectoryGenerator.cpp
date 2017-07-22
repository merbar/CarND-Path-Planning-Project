/* 
 * File:   polyTrajectoryGenerator.cpp
 * Author: merbar
 * 
 * Created on July 19, 2017, 10:51 PM
 */

#include "polyTrajectoryGenerator.h"

PolyTrajectoryGenerator::PolyTrajectoryGenerator() {
}

PolyTrajectoryGenerator::~PolyTrajectoryGenerator() {
}

// returns: trajectory for given number of timesteps (horizon) in Frenet coordinates
vector<vector<double>> PolyTrajectoryGenerator::generate_trajectory(vector<double> const &start, double max_speed, double horizon, vector<vector<double>> const &sensor_fusion) {
  const vector<double> start_s = {start[0], start[1], start[2]};
  const vector<double> start_d = {start[3], start[4], start[5]};
  const double dist_per_timestep = 0.0086 * max_speed;
  delta_s_maxspeed = horizon * dist_per_timestep;
  
  // figure out current lane
  // 0: left, 1: middle, 2: right
  int cur_lane_i = 0;
  if (start_d[0] > 8) cur_lane_i = 2;
  else if (start_d[0] > 4) cur_lane_i = 1;
  
  // generate goal points
  vector<vector<double>> goal_points;
  // go straight  
  double goal_s_pos = start_s[0] + delta_s_maxspeed;
  double goal_s_vel = dist_per_timestep * 50;
  double goal_s_acc = 0.0;
  double goal_d_pos = 2 + 4 * cur_lane_i;
  double goal_d_vel = 0.0;
  double goal_d_acc = 0.0;
  vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
  goal_points.push_back(goal_vec);
  //perturb_goal(goal_vec, goal_points);
  // change lane left
  // change lange right 
  
  /*
  vector<pair<vector<double>, vector<double>>> trajectory_coefficients;
  for (vector<double> goal : goal_points) {
    cout << "s goal: " << goal[0] << " " << goal[1] << " " << goal[2] << endl;
    cout << "d goal: " << goal[3] << " " << goal[4] << " " << goal[5] << endl;
    vector<double> goal_s = {goal[0], goal[1], goal[2]};
    vector<double> goal_d = {goal[3], goal[4], goal[5]};
    // ignore goal points that are out of bounds or exceed speed limit
    if ((goal[3] > 0.0) && (goal[3] < 12.0)) {      
      vector<double> traj_s = jmt(start, goal_s, horizon);
      vector<double> traj_d = jmt(start_d, goal, horizon);
      trajectory_coefficients.push_back(std::make_pair(traj_s, traj_d));
    }
  }
  // compute cost for each trajectory
  
  // choose least-cost trajectory and compute values for time horizon
  */
  
  vector<double> traj_s(horizon);
  vector<double> traj_d(horizon);
  // Just follow center lane
  for(int i = 0; i < horizon; i++) {
    //vector<double> new_xy = getXY(new_s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    traj_s.at(i) = start_s[0] + dist_per_timestep * i;
    traj_d.at(i) = 6;
  }
  vector<vector<double>> new_traj(2);
  new_traj[0] = traj_s;
  new_traj[1] = traj_d;
  
  return new_traj;
}


void PolyTrajectoryGenerator::perturb_goal(vector<double> goal, vector<vector<double>> &goal_points) {
  std::normal_distribution<double> distribution_s_pos(goal[0], delta_s_maxspeed / col_buf_length);
  std::normal_distribution<double> distribution_s_vel(goal[1], delta_s_maxspeed / col_buf_length / 3);
  std::normal_distribution<double> distribution_s_acc(goal[2], delta_s_maxspeed / col_buf_length / 6);
  std::normal_distribution<double> distribution_d_pos(goal[3], 0.5);
  std::normal_distribution<double> distribution_d_vel(goal[4], 0.5);
  std::normal_distribution<double> distribution_d_acc(goal[5], 0.5);
  vector<double> pert_goal(6);
  for (int i = 0; i < goal_perturb_samples; i++) {
    pert_goal.at(0) = distribution_s_pos(rand_generator);
    pert_goal.at(1) = distribution_s_vel(rand_generator);
    pert_goal.at(2) = distribution_s_acc(rand_generator);
    pert_goal.at(3) = distribution_d_pos(rand_generator);
    pert_goal.at(4) = distribution_d_vel(rand_generator);
    pert_goal.at(5) = distribution_d_acc(rand_generator);
    goal_points.push_back(pert_goal);
  }
}


vector<double> jmt(double start, double goal, int t) {
  /*
  double a_0 = 
  double a_1 = 
  double a_2 = start[0], start[1], start[2] / 2.0
    c_0 = a_0 + a_1 * T + 0.5 * a_2 * T**2
    c_1 = a_1 + a_2 * T
    c_2 = a_2
  Vector3d b(5.0, 6.0, 7.0);
  */
  vector<double> coeff(3);
  return coeff;
}


