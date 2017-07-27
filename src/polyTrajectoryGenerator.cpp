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

float PolyTrajectoryGenerator::exceeds_speed_limit_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion) {
  double cost = 0.0;
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_d(i) + traj.second.eval_d(i) > _hard_max_vel_per_timestep)
      cost = 1.0;
  }
  return cost;
}

float PolyTrajectoryGenerator::exceeds_accel_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion) {
  double cost = 0.0;
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_double_d(i) + traj.second.eval_double_d(i) > _hard_max_acc_per_timestep)
      cost = 1.0;
  }
  return cost;
}

float PolyTrajectoryGenerator::exceeds_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion) {
  double cost = 0.0;
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_triple_d(i) + traj.second.eval_triple_d(i) > _hard_max_jerk_per_timestep)
      cost = 1.0;
  }
  return cost;
}

float PolyTrajectoryGenerator::calculate_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<vector<double>> const &sensor_fusion) {
  Polynomial s = traj.first;
  Polynomial d = traj.second;
  double cost = 0.0;
  double ex_sp_lim_cost = exceeds_speed_limit_cost(traj, goal, sensor_fusion);
  double ex_acc_lim_cost = exceeds_accel_cost(traj, goal, sensor_fusion);
  double ex_jerk_lim_cost = exceeds_jerk_cost(traj, goal, sensor_fusion);
  cout << "COST FUNCTIONS" << endl;
  cout << "exceed speed limit: " << ex_sp_lim_cost << endl; 
  cout << "exceed acc limit: " << ex_acc_lim_cost << endl;
  cout << "exceed jerk limit: " << ex_jerk_lim_cost << endl;
  cost = ex_sp_lim_cost;
  return cost;
}

// returns: trajectory for given number of timesteps (horizon) in Frenet coordinates
vector<vector<double>> PolyTrajectoryGenerator::generate_trajectory(vector<double> const &start, double max_speed, double horizon, vector<vector<double>> const &sensor_fusion) {
  const vector<double> start_s = {start[0], start[1], start[2]};
  const vector<double> start_d = {start[3], start[4], start[5]};
  _horizon = horizon;
  _max_dist_per_timestep = 0.00894 * max_speed;
  
  _delta_s_maxspeed = horizon * _max_dist_per_timestep;
  // rough way to make the car accelerate more smoothly from a complete stop
  if (start_s[1] < _max_dist_per_timestep / 2.5) {
    _delta_s_maxspeed /= 2.0;
    _max_dist_per_timestep /= 2.0;
  }
  
  // figure out current lane
  // 0: left, 1: middle, 2: right
  int cur_lane_i = 0;
  if (start_d[0] > 8) cur_lane_i = 2;
  else if (start_d[0] > 4) cur_lane_i = 1;
  
  // #########################################
  // check which states make sense next
  // #########################################
  
  

  vector<vector<double>> goal_points;
  vector<vector<double>> traj_goals; // s, s_dot, s_double_dot, d, d_dot, d_double_dot
  vector<double> traj_costs;
  
  // #########################################
  // GENERATE GOALPOINTS: GO STRAIGHT
  // #########################################
  double goal_s_pos = start_s[0] + _delta_s_maxspeed;
  double goal_s_vel = _max_dist_per_timestep;
  double goal_s_acc = 0.0;
  double goal_d_pos = 2 + 4 * cur_lane_i;
  double goal_d_vel = 0.0;
  double goal_d_acc = 0.0;
  vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
  goal_points.push_back(goal_vec);
  //perturb_goal(goal_vec, goal_points);
  
  // #########################################
  // GENERATE GOALPOINTS: FOLLOW OTHER VEHICLE
  // #########################################
  
  // #########################################
  // GENERATE GOALPOINTS: CHANGE LANE LEFT
  // #########################################
  
  // #########################################
  // GENERATE GOALPOINTS: CHANGE LANE RIGHT
  // #########################################
  
  
  vector<pair<Polynomial, Polynomial>> trajectory_coefficients;
  for (vector<double> goal : goal_points) {
    cout << "s goal: " << goal[0] << " " << goal[1] << " " << goal[2] << endl;
    cout << "d goal: " << goal[3] << " " << goal[4] << " " << goal[5] << endl;
    vector<double> goal_s = {goal[0], goal[1], goal[2]};
    vector<double> goal_d = {goal[3], goal[4], goal[5]};
    // ignore goal points that are out of bounds
    if ((goal[3] > 1.0) && (goal[3] < 11.0)) {      
      Polynomial traj_s_poly = jmt(start_s, goal_s, horizon);
      Polynomial traj_d_poly = jmt(start_d, goal_d, horizon);
      trajectory_coefficients.push_back(std::make_pair(traj_s_poly, traj_d_poly));
      traj_goals.push_back({goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]});
    }     
  }
  
  // DEBUG
//  vector<double> deb_goal = goal_points[0];
//  Polynomial deb_traj_s = trajectory_coefficients[0].first;
//  Polynomial deb_traj_d = trajectory_coefficients[0].second;
  /*
  cout << endl;
  cout << "start s: " << start[0] << " " << start[1] << " " << start[2] << endl;
  cout << "start d: " << start[3] << " " << start[4] << " " << start[5] << endl;
  cout << "goal s: " << deb_goal[0] << " " << deb_goal[1] << " " << deb_goal[2] << endl;
  cout << "goal d: " << deb_goal[3] << " " << deb_goal[4] << " " << deb_goal[5] << endl;
  cout << "poly s: " << endl;
  deb_traj_s.print();
  cout << "poly d: " << endl;
  deb_traj_d.print();
  */
  
  // ################################
  // COMPUTE COST FOR EACH TRAJECTORY
  // ################################
  for (int i = 0; i < trajectory_coefficients.size(); i++) {
    double cost = calculate_cost(trajectory_coefficients[i], traj_goals[i], sensor_fusion);
    traj_costs.push_back(cost);
  }
  
  // choose least-cost trajectory
  
  // compute values for time horizon
  pair<Polynomial, Polynomial> rand_traj = trajectory_coefficients[0];
  vector<double> traj_s(horizon);
  vector<double> traj_d(horizon);
  for(int t = 0; t < horizon; t++) {
      traj_s[t] = rand_traj.first.eval(t);
      traj_d[t] = rand_traj.second.eval(t);
  }

// ###################################################  
// TEST - NAIVE LANE FOLLOW
// ###################################################
//  vector<double> traj_s(horizon);
//  vector<double> traj_d(horizon);
//  // Just follow center lane
//  for(int i = 0; i < horizon; i++) {
//    //vector<double> new_xy = getXY(new_s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//    traj_s.at(i) = start_s[0] + dist_per_timestep * i;
//    traj_d.at(i) = 10;
//  }
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// END - NAIVE LANE FOLLOW
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  
  vector<vector<double>> new_traj(2);
  new_traj[0] = traj_s;
  new_traj[1] = traj_d;
  
  return new_traj;
}
//
//double PolyTrajectoryGenerator::evaluate_poly(vector<double> coeff, double x) {
//    double result = 0.0;
//    for (int i = 0; i < coeff.size(); i++)
//        result += coeff[i] * pow(x, i);
//    return result;
//}


void PolyTrajectoryGenerator::perturb_goal(vector<double> goal, vector<vector<double>> &goal_points) {
  std::normal_distribution<double> distribution_s_pos(goal[0], _delta_s_maxspeed / 5.0);
  std::normal_distribution<double> distribution_s_vel(goal[1], _delta_s_maxspeed / 10.0);
  std::normal_distribution<double> distribution_s_acc(goal[2], _delta_s_maxspeed / 20.0);
  std::normal_distribution<double> distribution_d_pos(goal[3], 1.0);
  std::normal_distribution<double> distribution_d_vel(goal[4], 0.5);
  std::normal_distribution<double> distribution_d_acc(goal[5], 0.25);
  vector<double> pert_goal(6);
  for (int i = 0; i < _goal_perturb_samples; i++) {
    pert_goal.at(0) = distribution_s_pos(rand_generator);
    pert_goal.at(1) = distribution_s_vel(rand_generator);
    pert_goal.at(2) = distribution_s_acc(rand_generator);
    pert_goal.at(3) = distribution_d_pos(rand_generator);
    pert_goal.at(4) = distribution_d_vel(rand_generator);
    pert_goal.at(5) = distribution_d_acc(rand_generator);
    goal_points.push_back(pert_goal);
  }
}


Polynomial PolyTrajectoryGenerator::jmt(vector<double> const &start, vector<double> const &goal, int t) {
  double T = double(t);
  double t_2 = pow(T, 2);
  double t_3 = pow(T, 3);
  double t_4 = pow(T, 4);
  double t_5 = pow(T, 5);
  Eigen::Matrix3d A;
  A << t_3,   t_4,    t_5,
       3*t_2, 4*t_3,  5*t_4,
       6*t,   12*t_2, 20*t_3;
  
  double b_0 = start[0] + start[1] * t + 0.5 * start[2] * t_2;
  double b_1 = start[1] + start[2] * t;
  double b_2 = start[2];
  Eigen::MatrixXd b(3,1);
  b << goal[0] - b_0, goal[1] - b_1, goal[2] - b_2;
  
  Eigen::MatrixXd c = A.inverse() * b;
  //Eigen::Vector3d c = A.colPivHouseholderQr().solve(b);
  vector<double> coeff = {start[0], start[1], 0.5*start[2], c.data()[0], c.data()[1], c.data()[2]};
  Polynomial result(coeff);
  return result;
}

