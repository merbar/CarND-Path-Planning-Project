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

double PolyTrajectoryGenerator::exceeds_speed_limit_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_d(i) + traj.second.eval_d(i) > _hard_max_vel_per_timestep)
      return 1.0;
  }
  return 0.0;
}

double PolyTrajectoryGenerator::exceeds_accel_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_double_d(i) + traj.second.eval_double_d(i) > _hard_max_acc_per_timestep)
      return 1.0;
  }
  return 0.0;
}

double PolyTrajectoryGenerator::exceeds_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_triple_d(i) + traj.second.eval_triple_d(i) > _hard_max_jerk_per_timestep)
      return 1.0;
  }
  return 0.0;
}

double PolyTrajectoryGenerator::collision_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  for (int t = 0; t < _horizon; t++) {
    for (int i = 0; i < vehicles.size(); i++) {
      double ego_s = traj.first.eval(t);
      double ego_d = traj.second.eval(t);
      vector<double> traffic_state = vehicles[i].state_at(t); // {s,d}

      double dif_s = abs(traffic_state[0] - ego_s);
      double dif_d = abs(traffic_state[1] - ego_d);

      if ((dif_s <= _car_col_length) && (dif_d <= _car_col_width))
        return 1.0;
    }
  }
  return 0.0;
}

// adds cost for getting too close to another vehicle
double PolyTrajectoryGenerator::traffic_buffer_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    for (int i = 0; i < vehicles.size(); i++) {
      double ego_s = traj.first.eval(t);
      double ego_d = traj.second.eval(t);
      vector<double> traffic_state = vehicles[i].state_at(t); // {s,d}

      double dif_s = abs(traffic_state[0] - ego_s);
      double dif_d = abs(traffic_state[1] - ego_d);
      
      // if in the same lane and too close
      if ((dif_s <= _col_buf_length) && (dif_d <= 2.0))
        cost += 1 - logistic(dif_s);
    }
  }
  return cost;
}

// penalizes low average speeds compared to speed limit
double PolyTrajectoryGenerator::efficiency_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  double s_dist = goal[0] - traj.first.eval(0);
  double max_dist = _delta_s_maxspeed;
  return abs(logistic(max_dist - s_dist)); // abs() because going faster is actually bad
}

double PolyTrajectoryGenerator::total_accel_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    cost += abs(traj.first.eval_double_d(t));
    cost += abs(traj.second.eval_double_d(t));
  }
  return logistic(cost);
}

double PolyTrajectoryGenerator::total_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    cost += abs(traj.first.eval_triple_d(t));
    cost += abs(traj.second.eval_triple_d(t));
  }
  return logistic(cost);
}

double PolyTrajectoryGenerator::lane_depart_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    double ego_d = traj.second.eval(t);
    double lane_marking_proximity = fmod(ego_d, 4);
    if (lane_marking_proximity > 2.0)
      lane_marking_proximity = abs(lane_marking_proximity - 4);
    if (lane_marking_proximity <= _car_col_width) // car touches middle lane
      cost += 1 - logistic(lane_marking_proximity);
  }
  return cost;
}


double PolyTrajectoryGenerator::calculate_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  Polynomial s = traj.first;
  Polynomial d = traj.second;
    
  double cost = 0.0;
//  cout << "s poly:" << endl;
//  traj.first.print();
//  cout << "d poly:" << endl;
//  traj.second.print();

  // first situations that immediately make a trajectory infeasible
  double ex_sp_lim_cost = exceeds_speed_limit_cost(traj, goal, vehicles);
  double ex_acc_lim_cost = exceeds_accel_cost(traj, goal, vehicles);
  double ex_jerk_lim_cost = exceeds_jerk_cost(traj, goal, vehicles);
  double col_cost = collision_cost(traj, goal, vehicles);
//  cout << "COST FUNCTIONS" << endl;
//  cout << "exceed speed limit: " << ex_sp_lim_cost << endl; 
//  cout << "exceed acc limit: " << ex_acc_lim_cost << endl;
//  cout << "exceed jerk limit: " << ex_jerk_lim_cost << endl;
//  cout << "collision cost: " << col_cost << endl;
  
  double infeasible_costs = ex_sp_lim_cost + ex_acc_lim_cost + ex_jerk_lim_cost + col_cost;
  if (infeasible_costs > 0.0)
    return 999999;
  
  double tr_buf_cost   = traffic_buffer_cost(traj, goal, vehicles) * _cost_weights["tr_buf_cost"];
  double eff_cost      = efficiency_cost(traj, goal, vehicles) * _cost_weights["eff_cost"];
  double acc_cost      = total_accel_cost(traj, goal, vehicles) * _cost_weights["acc_cost"];
  double jerk_cost     = total_jerk_cost(traj, goal, vehicles) * _cost_weights["jerk_cost"];
  double lane_dep_cost = lane_depart_cost(traj, goal, vehicles) * _cost_weights["lane_dep_cost"];
  
//  cout << "traffic buffer cost: " << tr_buf_cost << endl;
//  cout << "efficiency cost: " << eff_cost << endl;
//  cout << "acceleration cost: " << acc_cost << endl;
//  cout << "jerk cost: " << jerk_cost << endl;
//  cout << "lane depart cost: " << lane_dep_cost << endl;
  
  cost = tr_buf_cost + eff_cost + acc_cost + jerk_cost + lane_dep_cost;
  return cost;
}

// returns a value between 0 and 1 for x in the range [0, infinity]
// and -1 to 1 for x in the range [-infinity, infinity].
double PolyTrajectoryGenerator::logistic(double x) {
    return (2.0 / (1 + exp(-x)) - 1.0);
}

// searches for closest vehicle in current travel lane. Returns index and s-distance.
int PolyTrajectoryGenerator::closest_vehicle_in_lane(vector<double> const &start, int ego_lane_i, vector<Vehicle> const &vehicles) {
  int closest_i = -1;
  float min_s_dif = 999;
  for (int i = 0; i < vehicles.size(); i++) {
    vector<double> traffic_d = vehicles[i].get_d();
    int traffic_lane_i = 0;
    if (traffic_d[0] > 8) traffic_lane_i = 2;
    else if (traffic_d[0] > 4) traffic_lane_i = 1;
    
    if (ego_lane_i == traffic_lane_i) {
      vector<double> traffic_s = vehicles[i].get_s();
      float dif_s = traffic_s[0] - start[0];
      if ((dif_s > 0.0) && (dif_s < min_s_dif)) {
        closest_i = i;
        min_s_dif = dif_s;
      }        
    }
  }
  cout << "closest vehicle s dif: " << min_s_dif  << " - index: " << closest_i << " - s value: " << vehicles[closest_i].get_s()[0] << endl;
  return closest_i;
}

// returns: trajectory for given number of timesteps (horizon) in Frenet coordinates
vector<vector<double>> PolyTrajectoryGenerator::generate_trajectory(vector<double> const &start, double max_speed, double horizon, vector<Vehicle> const &vehicles) {
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
  
  cout << "ego local s: " << start_s[0] << " d_vel: " << start_s[1] << " d: " << start_d[0] << endl;
  
  vector<vector<double>> goal_points;
  vector<vector<double>> traj_goals; // s, s_dot, s_double_dot, d, d_dot, d_double_dot
  vector<double> traj_costs;
  
  // #########################################
  // FIND FEASIBLE NEXT STATES FROM:
  // - go straight
  // - go straight following leading vehicle
  // - lane change left
  // - lane change right
  // #########################################
  bool go_straight = true;
  bool go_straight_follow_lead = false;
  bool change_left = false;
  bool change_right = false;
  // find closest vehicle ahead of us in current travel lane
  int closest_veh_i = closest_vehicle_in_lane(start, cur_lane_i, vehicles);
  
  if (closest_veh_i != -1) {
    bool traffic_is_close = false;
    vector<double> closest_veh_s = vehicles[closest_veh_i].get_s();
    if (abs(closest_veh_s[0] - start_s[0]) < _col_buf_length)
      traffic_is_close = true;
//    // predict if a collision will occur
//    bool col_occurs_with_closest_veh = false;
//    for (int t = 0; t < _horizon; t++) {
//      double ego_s = start_s[0] + start_s[1] * t;
//      vector<double> traffic_state = vehicles[closest_veh_i].state_at(t); // {s,d}
//      double dif_s = abs(traffic_state[0] - ego_s);
//      if (dif_s <= _car_col_length * 4) {
//        col_occurs_with_closest_veh = true;
//        break;
//      }
//    }
    cout << "closest veh i " << closest_veh_i << " - position s: " << vehicles[closest_veh_i].get_s()[0] << " - position d: " << vehicles[closest_veh_i].get_d()[0] << endl;
    if (traffic_is_close) {
      go_straight = false;
      go_straight_follow_lead = true;
      change_left = true;
      change_right = true;
    }
  }
    

  cout << "PLAN: ";
  if (go_straight)
    cout << " :GO STRAIGHT: ";
  if (go_straight_follow_lead)
    cout << " :FOLLOW LEAD: ";
  if (change_left)
    cout << " :CHANGE LEFT: ";
  if (change_right)
    cout << " :CHANGE RIGHT: ";
  cout << endl;
  
  // #########################################
  // GENERATE GOALPOINTS
  // #########################################
  // GO STRAIGHT
  if (go_straight) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    double goal_s_acc = 0.0;
    double goal_d_pos = 2 + 4 * cur_lane_i;
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    vector<vector<double>> goal_points_straight = {goal_vec};
    perturb_goal(goal_vec, goal_points_straight);
    // add to goal points
    goal_points.reserve(goal_points.size() + goal_points_straight.size());
    goal_points.insert(goal_points.end(),goal_points_straight.begin(),goal_points_straight.end());
  }
  
  // FOLLOW OTHER VEHICLE
  if (go_straight_follow_lead) {
    vector<double> lead_s = vehicles[closest_veh_i].get_s();
    
    double goal_s_pos = start_s[0] + lead_s[1] * _horizon;
    double goal_s_vel = lead_s[1];
    double goal_s_acc = 0.0;
    double goal_d_pos = 2 + 4 * cur_lane_i;
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    vector<vector<double>> goal_points_follow = {goal_vec};
    perturb_goal(goal_vec, goal_points_follow);
    // add to goal points
    goal_points.reserve(goal_points.size() + goal_points_follow.size());
    goal_points.insert(goal_points.end(),goal_points_follow.begin(),goal_points_follow.end());
  }
  
  // CHANGE LANE LEFT
  if (change_left) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    double goal_s_acc = 0.0;
    double goal_d_pos = (2 + 4 * cur_lane_i) - 4;
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    vector<vector<double>> goal_points_straight = {goal_vec};
    perturb_goal(goal_vec, goal_points_straight);
    // add to goal points
    goal_points.reserve(goal_points.size() + goal_points_straight.size());
    goal_points.insert(goal_points.end(),goal_points_straight.begin(),goal_points_straight.end());
  }
  
  // CHANGE LANE RIGHT
  if (change_right) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    double goal_s_acc = 0.0;
    double goal_d_pos = (2 + 4 * cur_lane_i) + 4;
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    vector<vector<double>> goal_points_straight = {goal_vec};
    perturb_goal(goal_vec, goal_points_straight);
    // add to goal points
    goal_points.reserve(goal_points.size() + goal_points_straight.size());
    goal_points.insert(goal_points.end(),goal_points_straight.begin(),goal_points_straight.end());
  }
  
  
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // END - GENERATE GOALPOINTS
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
  vector<pair<Polynomial, Polynomial>> trajectory_coefficients;
  for (vector<double> goal : goal_points) {
//    cout << "s goal: " << goal[0] << " " << goal[1] << " " << goal[2] << endl;
//    cout << "d goal: " << goal[3] << " " << goal[4] << " " << goal[5] << endl;
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
    double cost = calculate_cost(trajectory_coefficients[i], traj_goals[i], vehicles);
    traj_costs.push_back(cost);
  }
  
  // choose least-cost trajectory
  double min_cost = traj_costs[0];
  int min_cost_i = 0;
  for (int i = 1; i < traj_costs.size(); i++) {
    if (traj_costs[i] < min_cost) {
      min_cost = traj_costs[i];
      min_cost_i = i;
    }
  }
  
  cout << "cost: " << traj_costs[min_cost_i] << " - i: " << min_cost_i << endl;
  cout << "lowest cost traj goal s/d: " << goal_points[min_cost_i][0] << " : " << goal_points[min_cost_i][3] << endl;
  // ################################
  // COMPUTE VALUES FOR TIME HORIZON
  // ################################
  vector<double> traj_s(horizon);
  vector<double> traj_d(horizon);
  for(int t = 0; t < horizon; t++) {
      traj_s[t] = trajectory_coefficients[min_cost_i].first.eval(t);
      traj_d[t] = trajectory_coefficients[min_cost_i].second.eval(t);
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


// creates randomly generated variations of goal point
// it is not fully random: if we decrease distance in s, then velocity must also
// go down in order to create more feasible goals
void PolyTrajectoryGenerator::perturb_goal(vector<double> goal, vector<vector<double>> &goal_points) {
  double percentage_std_deviation = 0.1;
  std::normal_distribution<double> _distribution_10_percent(0.0, percentage_std_deviation);
//  std::normal_distribution<double> distribution_s_pos(goal[0], _delta_s_maxspeed / 10.0);
//  std::normal_distribution<double> distribution_s_vel(goal[1], 0.05);
//  std::normal_distribution<double> distribution_s_acc(goal[2], 0.02);
//  std::normal_distribution<double> distribution_d_pos(goal[3], 0.3);
//  std::normal_distribution<double> distribution_d_vel(goal[4], 0.01);
//  std::normal_distribution<double> distribution_d_acc(goal[5], 0.005);
  vector<double> pert_goal(6);
  cout << "perturbed: " << endl;
  for (int i = 0; i < _goal_perturb_samples; i++) {
    double multiplier = _distribution_10_percent(_rand_generator);
    pert_goal.at(0) = goal[0] + (_delta_s_maxspeed * multiplier);
    pert_goal.at(1) = goal[1] + (_max_dist_per_timestep * multiplier);
    pert_goal.at(2) = goal[2] * (1 + multiplier);
    
    multiplier = _distribution_10_percent(_rand_generator) * 0.25;
    pert_goal.at(3) = goal[3] + (2 * multiplier);
    pert_goal.at(4) = goal[4] * (1 + multiplier);
    pert_goal.at(5) = goal[5] * (1 + multiplier);;
//    pert_goal.at(0) = distribution_s_pos(_rand_generator);
//    pert_goal.at(1) = distribution_s_vel(_rand_generator);
//    pert_goal.at(2) = distribution_s_acc(_rand_generator);
//    pert_goal.at(3) = distribution_d_pos(_rand_generator);
//    pert_goal.at(4) = distribution_d_vel(_rand_generator);
//    pert_goal.at(5) = distribution_d_acc(_rand_generator);
    cout << pert_goal[0] << " : " <<  pert_goal[1] << " : " <<  pert_goal[2] << " : " <<  pert_goal[3] << " : " <<  pert_goal[4] << " : " <<  pert_goal[5] << endl;
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

