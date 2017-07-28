#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "polyTrajectoryGenerator.h"
#include "Vehicle.h"
#include "spline.h"
#include <cassert>

// FOR PLOTTING
//#include <ctime>
//#include <sstream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// FOR PLOTTING
bool do_plot = false;

// start position at end of lap to test transitions
//bool teleport_to_end = false;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
          double map_x = maps_x[i];
          double map_y = maps_y[i];
          double dist = distance(x,y,map_x,map_y);
          if(dist < closestLen)
          {
                  closestLen = dist;
                  closestWaypoint = i;
          }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
          closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> const &maps_x, vector<double> const &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
          prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
          frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
          frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
// in particular, uses splines instead of an estimated angle to project out into d, making the results much smoother
vector<double> getXY_splines(double s, double d, tk::spline const &spline_fit_s_to_x, tk::spline const &spline_fit_s_to_y, tk::spline const &spline_fit_s_to_dx, tk::spline const &spline_fit_s_to_dy) {
  double x_mid_road = spline_fit_s_to_x(s);
  double y_mid_road = spline_fit_s_to_y(s);
  double dx = spline_fit_s_to_dx(s);
  double dy = spline_fit_s_to_dy(s);

  double x = x_mid_road + dx * d;
  double y = y_mid_road + dy * d;

  return {x, y};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> const &maps_s, vector<double> const &maps_x, vector<double> const &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
          prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

//vector<double> get_frenet_state_from_path(vector<double> const &previous_path_x, vector<double> const &previous_path_y, vector<double> const &maps_x, vector<double> const &maps_y) {
//  assert(previous_path_x.size() >= 4);
//  // get angle from 0 to 1
//  double x_diff = previous_path_x[1] - previous_path_x[0];
//  double y_diff = previous_path_y[1] - previous_path_y[0];
//  double angle0 = atan2(y_diff, x_diff);
//  vector<double> frenet_0 = getFrenet(previous_path_x[0], previous_path_y[0], angle0, maps_x, maps_y);
//  
//  if (pow(x_diff,2) + pow(y_diff,2) < 0.001)
//    return {0.0, 0.0, 0.0, 0.0};
//  
//  x_diff = previous_path_x[2] - previous_path_x[1];
//  y_diff = previous_path_y[2] - previous_path_y[1];
//  double angle1 = atan2(y_diff, x_diff);
//  vector<double> frenet_1 = getFrenet(previous_path_x[1], previous_path_y[1], angle1, maps_x, maps_y);
//
//  x_diff = previous_path_x[3] - previous_path_x[2];
//  y_diff = previous_path_y[3] - previous_path_y[2];
//  double angle2 = atan2(y_diff, x_diff);
//  vector<double> frenet_2 = getFrenet(previous_path_x[2], previous_path_y[2], angle2, maps_x, maps_y);
//  
//  double s_vel_1 = frenet_1[0] - frenet_0[0];
//  double s_vel_2 = frenet_2[0] - frenet_1[0];
//  double s_acc = s_vel_2 - s_vel_1;
//  double d_vel_1 = frenet_1[1] - frenet_0[1];
//  double d_vel_2 = frenet_2[1] - frenet_1[1];
//  double d_acc = d_vel_2 - d_vel_1;
//  
//  
//  cout << endl;
//  cout << "x0: " << previous_path_x[0] << " x1: " << previous_path_x[1] << " x2: " << previous_path_x[2] << endl;
//  cout << "y0: " << previous_path_y[0] << " y1: " << previous_path_y[1] << " y2: " << previous_path_y[2] << endl;
//  cout << "a0: " << angle0 << " a1: " << angle1 << " a2: " << angle2 << endl;
//  cout << "s0: " << frenet_0[0] << " s1: " << frenet_1[0] << " s2: " << frenet_2[0] << endl;
//  cout << "d0: " << frenet_0[1] << " d1: " << frenet_1[1] << " d2: " << frenet_2[1] << endl;
//  cout << " s_vel: " << s_vel_1 << " s_acc: " << s_acc << " d_vel: " << d_vel_1 << " d_acc: " << d_acc << endl;
//  cout << endl;
//  return {s_vel_1, s_acc, d_vel_1, d_acc};
//}

void fit_spline_segment(double car_s, vector<double> const &map_waypoints_s, vector<double> const &map_waypoints_x, vector<double> const &map_waypoints_y, vector<double> const &map_waypoints_dx, vector<double> const &map_waypoints_dy, vector<double> &waypoints_segment_s, vector<double> &waypoints_segment_s_worldSpace, vector<double> &map_waypoints_x_upsampled, vector<double> &map_waypoints_y_upsampled, vector<double> &map_waypoints_s_upsampled, vector<double> &map_waypoints_dx_upsampled, vector<double> &map_waypoints_dy_upsampled, tk::spline &spline_fit_s_to_x, tk::spline &spline_fit_s_to_y, tk::spline &spline_fit_s_to_dx, tk::spline &spline_fit_s_to_dy) {
  // get 10 previous and 15 next waypoints
  vector<double> waypoints_segment_x, waypoints_segment_y, waypoints_segment_dx, waypoints_segment_dy;
  vector<int> wp_indeces;
  const int lower_wp_i = 9;
  const int upper_wp_i = 15;
  int prev_wp = -1;
  while(car_s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
          prev_wp++;
  for (int i = lower_wp_i; i > 0; i--) {
    if (prev_wp - i < 0)
      wp_indeces.push_back(map_waypoints_s.size() + (prev_wp - i));
    else
      wp_indeces.push_back((prev_wp - i) % map_waypoints_s.size());
  }
  wp_indeces.push_back(prev_wp);
  for (int i = 1; i < upper_wp_i; i++)
    wp_indeces.push_back((prev_wp + i) % map_waypoints_s.size());

  // FILL NEW SEGMENT WAYPOINTS
  const double max_s = 6945.554;
  bool crossed_through_zero = false;
  double seg_start_s = map_waypoints_s[wp_indeces[0]];
  for (int i = 0; i < wp_indeces.size(); i++) {
    int cur_wp_i = wp_indeces[i];
    waypoints_segment_x.push_back(map_waypoints_x[cur_wp_i]);
    waypoints_segment_y.push_back(map_waypoints_y[cur_wp_i]);
    waypoints_segment_dx.push_back(map_waypoints_dx[cur_wp_i]);
    waypoints_segment_dy.push_back(map_waypoints_dy[cur_wp_i]);
    // need special treatment of segments that cross over the end/beginning of lap
    if (i > 0) {
      if (cur_wp_i < wp_indeces[i-1])
        crossed_through_zero = true;
    }
    waypoints_segment_s_worldSpace.push_back(map_waypoints_s[cur_wp_i]);
    if (crossed_through_zero)
      waypoints_segment_s.push_back(abs(seg_start_s - max_s) + map_waypoints_s[cur_wp_i]);
    else
      waypoints_segment_s.push_back(map_waypoints_s[cur_wp_i] - seg_start_s);
  }

  //          cout << endl << "waypoints x" << endl;
  //          for (int i = 0; i < waypoints_segment_y.size(); i++) {
  //            cout << waypoints_segment_x[i] << ", ";
  //          }
  //          cout << endl << "waypoints y" << endl;
  //          for (int i = 0; i < waypoints_segment_y.size(); i++) {
  //            cout << waypoints_segment_y[i] << ", ";
  //          }
  //          cout << endl << "waypoints s : world s" << endl;
  //          for (int i = 0; i < waypoints_segment_y.size(); i++) {
  //            cout << waypoints_segment_s[i] << " : " << waypoints_segment_s_worldSpace[i] << endl;
  //          }
  //          cout << endl;
  
//  cout << "wp indeces:" << endl;
//  for (auto val : wp_indeces)
//    cout << val << ", ";
//  cout << endl << endl;
//  
//  cout << "wp world s:" << endl;
//  for (auto val : waypoints_segment_s_worldSpace)
//    cout << val << ", ";
//  cout << endl << endl;
//  
//  cout << "wp segment s:" << endl;
//  for (auto val : waypoints_segment_s)
//    cout << val << ", ";
//  cout << endl << endl;
  
  // UPSAMPLING - 1 meter per sample
  // need to fit splines first to smooth out the path
//  tk::spline spline_fit_s_to_x;
//  tk::spline spline_fit_s_to_y;
//  tk::spline spline_fit_s_to_dx;
//  tk::spline spline_fit_s_to_dy;
  spline_fit_s_to_x.set_points(waypoints_segment_s, waypoints_segment_x);
  spline_fit_s_to_y.set_points(waypoints_segment_s, waypoints_segment_y);
  spline_fit_s_to_dx.set_points(waypoints_segment_s, waypoints_segment_dx);
  spline_fit_s_to_dy.set_points(waypoints_segment_s, waypoints_segment_dy);

  const int samples = int(waypoints_segment_s[waypoints_segment_s.size()-1]);
  map_waypoints_x_upsampled.reserve(samples);
  map_waypoints_y_upsampled.reserve(samples);
  map_waypoints_s_upsampled.reserve(samples);
  map_waypoints_dx_upsampled.reserve(samples);
  map_waypoints_dy_upsampled.reserve(samples);
  for (int i = 0; i < samples; i++) {
    map_waypoints_x_upsampled.push_back(spline_fit_s_to_x(i));
    map_waypoints_y_upsampled.push_back(spline_fit_s_to_y(i));
    map_waypoints_s_upsampled.push_back(i);
    map_waypoints_dx_upsampled.push_back(spline_fit_s_to_dx(i));
    map_waypoints_dy_upsampled.push_back(spline_fit_s_to_dy(i));
  }
}


  // converts world space s coordinate to local space based on provided mapping
double get_local_s(double world_s, vector<double> const &waypoints_segment_s_worldSpace, vector<double> const &waypoints_segment_s) {
  int prev_wp = 0;
  // special case: first wp in list is larger than s. Meaning we are crossing over 0 somewhere.
  // go to index with value zero first and search from there.
  if (waypoints_segment_s_worldSpace[0] > world_s) {
      while (waypoints_segment_s_worldSpace[prev_wp] != 0.0)
          prev_wp += 1;
  }
  while ((waypoints_segment_s_worldSpace[prev_wp+1] < world_s) && (waypoints_segment_s_worldSpace[prev_wp+1] != 0))
      prev_wp += 1;
  double diff_world = world_s - waypoints_segment_s_worldSpace[prev_wp];
  double s_local = waypoints_segment_s[prev_wp] + diff_world;
  return s_local;
}


void plot(double car_s, double car_yaw, vector<double> const &next_x_vals, vector<double> const &next_y_vals, vector<double> const &map_waypoints_x_upsampled, vector<double> const &map_waypoints_y_upsampled, vector<vector<double>> const &sensor_fusion) {
  int track_s_lookahead = 200;
  vector<double> track_s = {car_s, car_s + 300};
  const vector<double> lane_mark_1 = {0, 0};
  const vector<double> lane_mark_2 = {4, 4};
  const vector<double> lane_mark_3 = {8, 8};
  const vector<double> lane_mark_4 = {12, 12};
  const vector<vector<double>> lane_markings = {lane_mark_1,lane_mark_2,lane_mark_3,lane_mark_4};
  vector<double> traj_s(next_x_vals.size());
  vector<double> traj_d(next_x_vals.size());
  for (int i = 0; i < next_x_vals.size(); i++) {
      vector<double> s_d = getFrenet(next_x_vals[i], next_y_vals[i], deg2rad(car_yaw), map_waypoints_x_upsampled, map_waypoints_y_upsampled);
      traj_s[i] = s_d[0];
      traj_d[i] = s_d[1];
  }

  plt::ylim(12, 0);
  // three lanes in travel direction
  for (auto marking : lane_markings) {
    plt::plot(track_s, marking, "r");
  }
  // other cars
  for (auto veh : sensor_fusion) {
    plt::plot({veh[5]}, {veh[6]}, "b+");
  }
  // current trajectory
  // this is still buggy, but in the ballpark
  plt::plot(traj_s, traj_d, "g");
  std::string filename = "../img/plot_";
  filename += std::to_string(100);
  plt::xlim(car_s-10, car_s+track_s_lookahead+10);
  plt::save(filename);
  plt::clf();
}

            
int main() {
  uWS::Hub h;
  
  // FOR PLOTTING
  clock_t cur_time = clock();
  clock_t last_plot_time = clock();
  int plot_i = 10000;
  if (do_plot) {
    plt::figure();
  }
   
  PolyTrajectoryGenerator PTG;
  
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  // create object for ego vehicle;
  Vehicle ego_veh;
  
  // smooth out waypoints
  // fit splines to x and y in relation to s
//  tk::spline spline_fit_s_to_x;
//  tk::spline spline_fit_s_to_y;
  // spline boundaries are linear, so we need to join the two ends
  // add point close to end of lap to hook up smoothly with beginning of new lap
//  map_waypoints_s.push_back(6945.554);
//  map_waypoints_x.push_back(784.6001);
//  map_waypoints_y.push_back(1135.571);
//  map_waypoints_s.push_back(6976.228478531);
//  map_waypoints_x.push_back(815.2679);
//  map_waypoints_y.push_back(1134.93);
  
//  spline_fit_s_to_x.set_points(map_waypoints_s, map_waypoints_x);
//  spline_fit_s_to_y.set_points(map_waypoints_s, map_waypoints_y);
  
  // resample map waypoints themselves. one point per meter.
//  const int samples = 6945;
//  vector<double> map_waypoints_x_upsampled(samples);
//  vector<double> map_waypoints_y_upsampled(samples);
//  vector<double> map_waypoints_s_upsampled(samples);
//  for (int i = 0; i < samples; i++) {
//    map_waypoints_x_upsampled[i] = spline_fit_s_to_x(i);
//    map_waypoints_y_upsampled[i] = spline_fit_s_to_y(i);
//    map_waypoints_s_upsampled[i] = i;
//  }
    

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,
            &map_waypoints_dy,&cur_time,&last_plot_time,&plot_i,&PTG,&ego_veh]
            (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          
//          cout << "s: " << car_s << " d: " << car_d << " x: " << car_x << " y: " << car_y << " speed " << car_speed << endl;
          
          // update actual position
          ego_veh.set_frenet_pos(car_s, car_d);

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          int prev_path_size = previous_path_x.size();
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          
          
//          if (prev_path_size > 3) {
//            vector<double> frenet_state = get_frenet_state_from_path(previous_path_x, previous_path_y, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
//            car_s_vel = frenet_state[0];
//            car_s_acc = frenet_state[1];
//            car_d_vel = frenet_state[2];
//            car_d_acc = frenet_state[3];
//          }
          
          // get ego vel and acc in frenet space
//          if (prev_path_size > 3) {
//              double<vector> frenet_0 = getXY(s, d, spline_fit_s_to_x, spline_fit_s_to_y);
//          }  
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          // END - Get ego vehicle velocity and acceleration in s and d
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          
          vector<double> next_x_vals;
          vector<double> next_y_vals; 
                    
          // ###################################################  
          // TEST - Just follow given lane and smooth paths
          // ###################################################
//          double d = 10.0;
//          int horizon = 250;
//          int update_interval = 100; // update every two seconds
////          cout << "elapsed timesteps: " << horizon - previous_path_x.size() << endl;
//          if (previous_path_x.size() < horizon - update_interval) {
//            cout << "update" << endl;
//            
//            double dist_inc = 0.4;
//            for(int i = 0; i < horizon; i++) {
//              double s = car_s + dist_inc*i;
//              //vector<double> xy = getXY(s, d, spline_fit_s_to_x, spline_fit_s_to_y);
//              vector<double> xy = getXY(s, d, map_waypoints_s_upsampled, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
//
//              // SMOOTHING
//              double smooth_range = 100;
//              if ((prev_path_size >= smooth_range) && (i < smooth_range)) {
////                  if (i < smooth_range) {     
////                    cout << xy[0] << " :ps: " << xy[1] << endl;
////                    cout << previous_path_x[i] << " :pp: " << previous_path_y[i] << endl;
////                  }
//                  double scaleFac = i / smooth_range;
//                  xy[0] = scaleFac * xy[0] + (1 - scaleFac) * double(previous_path_x[i]);
//                  xy[1] = scaleFac * xy[1] + (1 - scaleFac) * double(previous_path_y[i]);
//              }
//              
//              next_x_vals.push_back(xy[0]);
//              next_y_vals.push_back(xy[1]);
//            }
////            cout << "smoothed:" << endl;
////            for (int i = 0; i < 10; i++)                
////                cout << next_x_vals[i] << " : " << next_y_vals[i] << endl;
//          } else {
//              for(int i = 0; i < previous_path_x.size(); i++) {
//                next_x_vals.push_back(previous_path_x[i]);
//                next_y_vals.push_back(previous_path_y[i]);
//              }
//          }
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          // END - Just follow center lane and smooth paths
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          
          
          // ###################################################  
          // TEST - Just follow given lane and use new path as offset to previous
          // ###################################################
//          double d = 10.0;
//          int horizon = 200;
//          int update_interval = 100; // update every two seconds
//          double dist_inc = 0.4;
////          cout << "elapsed timesteps: " << horizon - previous_path_x.size() << endl;
//          bool smooth_path = previous_path_x.size() > 0;
//          if (previous_path_x.size() < horizon - update_interval) {      
//            cout << "update" << endl;
//            
//            // extract surrounding waypoints and fit a spline
//            vector<double> waypoints_segment_x, waypoints_segment_y, waypoints_segment_s;
//            vector<double> waypoints_segment_s_worldSpace;
//            vector<double> map_waypoints_x_upsampled, map_waypoints_y_upsampled, map_waypoints_s_upsampled;
//            fit_spline_segment(car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y, waypoints_segment_x, waypoints_segment_y, waypoints_segment_s, waypoints_segment_s_worldSpace, map_waypoints_x_upsampled, map_waypoints_y_upsampled, map_waypoints_s_upsampled);
//            
//            double prev_new_x, prev_new_y;
//            // TODO: convert current car_s into our local Frenet space
//            double local_s = get_local_s(car_s, waypoints_segment_s_worldSpace, waypoints_segment_s);
//            // start with current car position in x/y
//            vector<double> prev_xy_planned = getXY(local_s, d, map_waypoints_s_upsampled, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
//            if (smooth_path) {
//              // re-use first point of previous path
//              next_x_vals.push_back(previous_path_x[0]);
//              next_y_vals.push_back(previous_path_y[0]);
//              prev_new_x = previous_path_x[0];
//              prev_new_y = previous_path_y[0];
//            } else {
//              next_x_vals.push_back(prev_xy_planned[0]);
//              next_y_vals.push_back(prev_xy_planned[1]);
//            }
//            for(int i = 1; i < horizon; i++) {
//              double s = local_s + dist_inc * i;
//              vector<double> xy_planned = getXY(s, d, map_waypoints_s_upsampled, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
//              if (smooth_path) {
//                double x_dif_planned =  xy_planned[0] - prev_xy_planned[0];
//                double y_dif_planned =  xy_planned[1] - prev_xy_planned[1];
//                prev_new_x = prev_new_x + x_dif_planned;
//                prev_new_y = prev_new_y + y_dif_planned;
//                next_x_vals.push_back(prev_new_x);
//                next_y_vals.push_back(prev_new_y);
//                prev_xy_planned = xy_planned;
//                
//              } else {
//                next_x_vals.push_back(xy_planned[0]);
//                next_y_vals.push_back(xy_planned[1]);
//              }
//            }
//
//          } else {
//              for(int i = 0; i < previous_path_x.size(); i++) {
//                next_x_vals.push_back(previous_path_x[i]);
//                next_y_vals.push_back(previous_path_y[i]);
//              }
//          }
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          // END - Just follow given lane and use new path as offset to previous
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          

          // ###################################################  
          // TEST - Just go straight
          // ###################################################
//          double dist_inc = 0.5;
//          for(int i = 0; i < 50; i++)
//          {
//            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
//          }
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          // END - Just go straight
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          
          
          // ###################################################  
          // PATH PLANNING
          // ###################################################
                   
//          cout << "prev path size: " << previous_path_x.size() << endl;

          int horizon = 200;
          bool smooth_path = previous_path_x.size() > 0;
//          double smooth_range = 100;
          int update_interval = 100; // update every two seconds
//          cout << "prev path:" << endl;
//          for (int i = 0; i < prev_path_size; i++) {
//              cout << "x: " << previous_path_x[i] << " :: y: " << previous_path_y[i] << endl;
//          }
          
//          double car_speed_per_timestep = car_speed * 0.00894; // 0.44704 * 0.02;
          
//          if (teleport_to_end) {
//              cout << "teleporting" << endl;
//              car_state[0] = 6500;
//              teleport_to_end = false;
//          }

          if (previous_path_x.size() < horizon - update_interval) {
            cout << endl;
            cout << "PATH UPDATE" << endl;
            
            // extract surrounding waypoints and fit a spline
            vector<double> waypoints_segment_s;
            vector<double> waypoints_segment_s_worldSpace;
            // TODO: Clean most (all?) of these up! Change signature of fit_spline_segment as well.
            vector<double> map_waypoints_x_upsampled, map_waypoints_y_upsampled, map_waypoints_s_upsampled, map_waypoints_dx_upsampled, map_waypoints_dy_upsampled;
            tk::spline spline_fit_s_to_x;
            tk::spline spline_fit_s_to_y;
            tk::spline spline_fit_s_to_dx;
            tk::spline spline_fit_s_to_dy;
            fit_spline_segment(car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, waypoints_segment_s, waypoints_segment_s_worldSpace, map_waypoints_x_upsampled, map_waypoints_y_upsampled, map_waypoints_s_upsampled, map_waypoints_dx_upsampled, map_waypoints_dy_upsampled, spline_fit_s_to_x, spline_fit_s_to_y, spline_fit_s_to_dx, spline_fit_s_to_dy);
            
            // convert current car_s into our local Frenet space
            double car_local_s = get_local_s(car_s, waypoints_segment_s_worldSpace, waypoints_segment_s);
            // convert sensor fusion data into local Frenet space
            for (int i = 0; i < sensor_fusion.size(); i++) {
              sensor_fusion[i][5] = get_local_s(sensor_fusion[i][5], waypoints_segment_s_worldSpace, waypoints_segment_s);
            }
            // turn sensor fusion data into Vehicle objects
            vector<Vehicle> envir_vehicles(sensor_fusion.size());
            for (int i = 0; i < sensor_fusion.size(); i++) {
              envir_vehicles[i].set_frenet_pos(sensor_fusion[i][5], sensor_fusion[i][6]);
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double velocity_per_timestep = sqrt(pow(vx, 2) + pow(vy, 2)) / 50.0;
              envir_vehicles[i].set_frenet_motion(velocity_per_timestep, 0.0, 0.0, 0.0);
            }
            
            // get last known car state
            vector<double> prev_car_s = ego_veh.get_s();
            vector<double> prev_car_d = ego_veh.get_d();            
            // collect best guess at current car state. S position in local segment space
            vector<double> car_state = {car_local_s, prev_car_s[1], prev_car_s[2], car_d, prev_car_d[1], prev_car_d[2]};
            
            // PLAN NEW PATH
//            cout << "planning path" << endl;
//            cout << "LOCAL car state - s: " << car_state[0] << " : " << car_state[1] << " : " << car_state[2] << " # d: " << car_state[3] << " : " << car_state[4] << " : " << car_state[5] << endl;
            double speed_limit = 45;
            vector<vector<double>> new_path = PTG.generate_trajectory(car_state, speed_limit, horizon, envir_vehicles);
            
            // ###################################################  
            // store ego vehicle velocity and acceleration in s and d for next cycle
            // ###################################################
            // make a bold prediction into the future
//            cout << "new path sizes: "  << new_path.size() << " - " << new_path[0].size() << " : " << new_path[1].size() << endl;

            double s0 = new_path[0][update_interval + 2];
            double s1 = new_path[0][update_interval + 3];
            double s2 = new_path[0][update_interval + 4];
            double d0 = new_path[1][update_interval + 2];
            double d1 = new_path[1][update_interval + 3];
            double d2 = new_path[1][update_interval + 4];
            double s_v1 = s1 - s0;
            double s_v2 = s2 - s1;
            double s_v = (s_v1 + s_v2) * 0.5;
            double s_a = s_v2 - s_v1;
            double d_v1 = d1 - d0;
            double d_v2 = d2 - d1;
            double d_v = (d_v1 + d_v2) * 0.5;
            double d_a = d_v2 - d_v1;          
            ego_veh.set_frenet_motion(s_v, s_a, d_v, d_a);
//            cout << "pred: " << s_v << " : " << s_a << " -- " << d_v << " : " << d_a << endl;
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  
            // END - store ego vehicle velocity and acceleration in s and d for next cycle
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            
//            cout << "first 5 of planned path in Frenet: " << endl;
//            for (int i = 0; i < 5; i++)
//              cout << new_path[0][i] << " : " << new_path[1][i] << endl;
            
            double new_x, new_y;
            // start with current car position in x/y
            //vector<double> prev_xy_planned = getXY(new_path[0][0], new_path[1][0], map_waypoints_s_upsampled, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
            vector<double> prev_xy_planned = getXY_splines(new_path[0][0], new_path[1][0], spline_fit_s_to_x, spline_fit_s_to_y, spline_fit_s_to_dx, spline_fit_s_to_dy);
            if (smooth_path) {
              // re-use first point of previous path
              new_x = previous_path_x[0];
              new_y = previous_path_y[0];
              next_x_vals.push_back(new_x);
              next_y_vals.push_back(new_y);
              
            } else {
              next_x_vals.push_back(prev_xy_planned[0]);
              next_y_vals.push_back(prev_xy_planned[1]);
            }
            
//            cout << "dx: " << spline_fit_s_to_dx(car_local_s) << " - dy: " << spline_fit_s_to_dy(car_local_s) << endl;
            
            for(int i = 1; i < horizon; i++) {
              //vector<double> xy_planned = getXY(new_path[0][i], new_path[1][i], map_waypoints_s_upsampled, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
              vector<double> xy_planned = getXY_splines(new_path[0][i], new_path[1][i], spline_fit_s_to_x, spline_fit_s_to_y, spline_fit_s_to_dx, spline_fit_s_to_dy);
              if (smooth_path) {
                double x_dif_planned =  xy_planned[0] - prev_xy_planned[0];
                double y_dif_planned =  xy_planned[1] - prev_xy_planned[1];
                new_x = new_x + x_dif_planned;
                new_y = new_y + y_dif_planned;
                next_x_vals.push_back(new_x);
                next_y_vals.push_back(new_y);
                prev_xy_planned = xy_planned;
                
              } else {
                next_x_vals.push_back(xy_planned[0]);
                next_y_vals.push_back(xy_planned[1]);
              }
            }

          } else {
//            cout << "PATH REUSE" << endl;
            for(int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          }

          
//          if (previous_path_x.size() < horizon - update_interval) {
//            cout << "updating path" << endl;
//            vector<vector<double>> new_path = PTG.generate_trajectory(car_state, 45, horizon, sensor_fusion);
//            // new_path is "ideal" path in Frenet coordinates. Skip first element because it pops.
//            for (int i = 0; i < new_path[0].size(); i++) {
//              vector<double> new_path_xy = getXY(new_path[0][i], new_path[1][i], map_waypoints_s_upsampled, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
////              cout << "s: " << new_path[0][i] << " d: " << new_path[1][i] << endl;
//              // smoothing
//              if ((prev_path_size >= smooth_range) && (i < smooth_range)) {
//                  double scaleFac = i / smooth_range;
//                  new_path_xy[0] = scaleFac * new_path_xy[0] + (1 - scaleFac) * double(previous_path_x[i-1]);
//                  new_path_xy[1] = scaleFac * new_path_xy[1] + (1 - scaleFac) * double(previous_path_y[i-1]);
//              }
//
//              next_x_vals.push_back(new_path_xy[0]);
//              next_y_vals.push_back(new_path_xy[1]);
//            }
//          } else {
//            for(int i = 0; i < previous_path_x.size(); i++) {
//                next_x_vals.push_back(previous_path_x[i]);
//                next_y_vals.push_back(previous_path_y[i]);
//              }
//          }
//          cout << endl;
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          // END - PATH PLANNING
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//          cout << "first 5 of sent path in x/y: " << endl;
//            for (int i = 0; i < 5; i++)
//              cout << next_x_vals[i] << " : " << next_y_vals[i] << endl;
//          cout << "sent path sizes: " << next_x_vals.size() << " : " << next_y_vals.size() << endl;
//          cout << endl;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
