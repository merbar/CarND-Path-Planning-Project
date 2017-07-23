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
#include "spline.h"

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
bool teleport_to_end = false;

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
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, tk::spline &spline_fit_x, tk::spline &spline_fit_y) {
    // needed to dial this way down to achieve more stable trajectories
    double seg_length = 0.001;
    
    double x_cur = spline_fit_x(s);
    double y_cur = spline_fit_y(s);
    double x_next = spline_fit_x(s + seg_length);
    double y_next = spline_fit_y(s + seg_length);
    // point at center of road. Now need to offset for d
    double heading = atan2((y_next - y_cur),(x_next - x_cur));
    // the x,y along the segment
    double seg_x = x_cur + seg_length * cos(heading);
    double seg_y = y_cur + seg_length * sin(heading);

    double perp_heading = heading - pi() / 2.0;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);    

    return {x, y};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY_from_wp(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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
  
  // smooth out waypoints
  // fit splines to x and y in relation to s
  tk::spline spline_fit_x;
  tk::spline spline_fit_y;
  // spline boundaries are linear, so we need to join the two ends
  // add point close to end of lap to hook up smoothly with beginning of new lap
  map_waypoints_s.push_back(6945.554);
  map_waypoints_x.push_back(784.55);
  map_waypoints_y.push_back(1135.56);
  spline_fit_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_fit_y.set_points(map_waypoints_s, map_waypoints_y);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,
            &map_waypoints_dy,&cur_time,&last_plot_time,&plot_i,&PTG,&spline_fit_x,&spline_fit_y]
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

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          int prev_path_size = previous_path_x.size();
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          double car_s_vel = 0;
          double car_s_acc = 0.0;
          double car_d_vel = 0.0;
          double car_d_acc = 0.0;
          
          // get ego vel and acc in frenet space
//          if (prev_path_size > 0) {
//              double<vector> frenet_0 = getXY(s, d, spline_fit_x, spline_fit_y);
//          }

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
                    
          /*
          // TESTS
          // Just follow center lane
          double dist_inc = 0.4;
          for(int i = 0; i < 50; i++) {
            double new_s = car_s + dist_inc * i;
            vector<double> new_xy = getXY(new_s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(new_xy[0]);
            next_y_vals.push_back(new_xy[1]);
          }
          */
          /*
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          double dist_inc = 0.5;
          for(int i = 0; i < 50; i++)
          {
            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          }
          */
          
          // plan the path!!
          //cout << endl;
          //cout << "car s: " << car_s << " car x: " << car_x << " car y: " << car_y << " car speed: " << car_speed << " car d: " << car_d << endl;
          //vector<double> bla = getXY_from_wp(6945, 0.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //cout << bla[0] << " " << bla[1] << endl;
          int horizon = 10;
          
//          cout << "prev path:" << endl;
//          for (int i = 0; i < prev_path_size; i++) {
//              cout << "x: " << previous_path_x[i] << " :: y: " << previous_path_y[i] << endl;
//          }

          vector<double> car_state = {car_s, car_speed, 0.0, car_d, 0.0, 0.0};
          if (teleport_to_end) {
              cout << "teleporting" << endl;
              car_state[0] = 6500;
              teleport_to_end = false;
          }
//          vector<vector<double>> new_path = PTG.generate_trajectory(car_state, 45, horizon, sensor_fusion);
//          // new_path is "ideal" path in Frenet coordinates. Skip first element because it pops.
//          for (int i = 1; i < new_path[0].size(); i++) {
//            vector<double> new_path_xy = getXY(new_path[0][i], new_path[1][i], spline_fit_x, spline_fit_y);
//            //vector<double> new_path_xy = getXY_from_wp(new_path[0][i], new_path[1][i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
//            
//            // smooth over next 10(?) samples with prev path
//            double smooth_range = 0.0;
//            if ((prev_path_size > 0) && (i < smooth_range)) {
//                double scaleFac = i / smooth_range;
//                new_path_xy[0] = scaleFac * new_path_xy[0] + (1 - scaleFac) * double(previous_path_x[i-1]);
//                new_path_xy[1] = scaleFac * new_path_xy[1] + (1 - scaleFac) * double(previous_path_y[i-1]);
//            }
//            
//            next_x_vals.push_back(new_path_xy[0]);
//            next_y_vals.push_back(new_path_xy[1]);
//          }
          
          
          
          if (previous_path_x.size() < 3) {
              //cout << "update" << endl;
            double dist_inc = 0.4;
            for(int i = 1; i < horizon; i++) {
              double s = car_s + dist_inc*i;
              double d = 10.0;
              vector<double> xy = getXY(s, d, spline_fit_x, spline_fit_y);
              //vector<double> xy = getXY_from_wp(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
//              if (i < 5)
//                  cout << xy[0] << " : " << xy[1] << endl;
            }
          } else {
              for(int i = 0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }
          }
          
         
          cout << "s: " << car_s << " d: " << car_d << " x: " << car_x << " y: " << car_y << " speed " << car_speed << endl;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // #################################################
          // PLOTTING
          if (do_plot) {
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
                vector<double> s_d = getFrenet(next_x_vals[i], next_y_vals[i], deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
                traj_s[i] = s_d[0];
                traj_d[i] = s_d[1];
            }
                    
            cur_time = clock();
            if (((cur_time - last_plot_time) / float(CLOCKS_PER_SEC)) > 0.005) {
              plt::ylim(12, 0);
              // three lanes in travel direction
              for (auto marking : lane_markings) {
                plt::plot(track_s, marking, "r");
              }
              // other cars
              for (auto veh : sensor_fusion) {
                plt::plot(veh[5], veh[6], "b+");
              }
              // current trajectory
              // this is still buggy, but in the ballpark
              plt::plot(traj_s, traj_d, "g");
              std::string filename = "../img/plot_";
              filename += std::to_string(plot_i);
              plt::xlim(car_s-10, car_s+track_s_lookahead+10);
              plt::save(filename);
              plt::clf();
              plot_i = plot_i + 1;
              last_plot_time = clock();
            }
          }
          // END PLOTTING
          // #################################################
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
















































































