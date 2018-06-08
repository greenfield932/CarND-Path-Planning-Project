#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "json.hpp"
#include "spline.h"
#include "helpers.h"
#include "vehicle.h"
using namespace std;


// for convenience
using json = nlohmann::json;

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



int main() {
  uWS::Hub h;

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

	int lane = 1;
	double ref_vel = 49.5;


  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
		        cout<<"speed="<<car_speed<<" s="<<car_s<<" d="<<car_d<<" yaw="<<car_yaw<<" x="<<car_x<<" y="<<car_y<<endl;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	
		double dist_inc = 0.3;
		int horizon = 2;
		double speed_m_s = car_speed*1.60934/3600.;
		double speed_limit_mph = 49.5;
		double speed_limit_m_s = speed_limit_mph*1.60934/3600.;
		vector<double> jmt_s = JMT({0, speed_m_s, 0}, {30, speed_limit_m_s, 3.0}, horizon);
//		vector<double> jmt_s = JMT({car_d, 0, 0}, {car_d, 49.5, 0.}, 5);

		double time_step = 0.02;

		/*for(int i = 0; i < 50; i++)
		{
		
			double next_s = car_s + (i+1)*dist_inc;
			double next_d = 6;
			
			vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//                  next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
			//                  next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
			next_x_vals.push_back(xy[0]);
			next_y_vals.push_back(xy[1]);
		}*/
	
		/*for(int i = 0; i < double(horizon)/time_step; i++)
		{
		
			double t = (i+3)*time_step;
			double next_s = car_s + jmt_s[0] + jmt_s[1] * t + jmt_s[2] * t*t + jmt_s[3] * t*t*t + jmt_s[4] * t*t*t*t + jmt_s[5] * t*t*t*t*t;
			double next_d = 6;
			
			vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//                  next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
			//                  next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
			next_x_vals.push_back(xy[0]);
			next_y_vals.push_back(xy[1]);
		}*/
    Vehicle vehicle(car_s, car_d, car_yaw, car_speed, car_x, car_y);
    vehicle.process(map_waypoints_x, map_waypoints_y, map_waypoints_s, previous_path_x, previous_path_y);
          
    next_x_vals = vehicle.ptsx();
    next_y_vals = vehicle.ptsy();
          
		/*vector<double> ptsx, ptsy;
		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		int prev_size = previous_path_x.size();
		cout<<"prev size:"<<prev_size<<endl;
		if(prev_size < 2)
		{

			double prev_car_x = car_x - cos(car_yaw);
			double prev_car_y = car_y - sin(car_yaw); 
			ptsx.push_back(prev_car_x);
			ptsy.push_back(prev_car_y);
			ptsx.push_back(car_x);
			ptsy.push_back(car_y);
		}	
		else{
			ref_x = previous_path_x[prev_size - 1];
			ref_y = previous_path_y[prev_size - 1];
			double ref_x_prev = previous_path_x[prev_size - 2];
			double ref_y_prev = previous_path_y[prev_size - 2];
			ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

			ptsx.push_back(ref_x_prev);
			ptsx.push_back(ref_x);
			ptsy.push_back(ref_y_prev);
			ptsy.push_back(ref_y);

		}

		vector<double> next_wp0 = getXY(car_s + 30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s + 60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s + 90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);

		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);

		for(int i=0;i<ptsx.size();++i){
			double shift_x = ptsx[i] - ref_x;
			double shift_y = ptsy[i] - ref_y;
			ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
			ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
			//cout<<i<<" ptsx:"<<ptsx[i]<<" ptsy:"<<ptsy[i]<<endl;
		}
		vector<double> ptsx2, ptsy2;
		ptsx2.push_back(ptsx[0]);
		ptsy2.push_back(ptsy[0]);
		for(int i=1;i<ptsx.size();++i){
			if(fabs(ptsx[i]-ptsx[i-1])<1e-5){
				continue;
			}
			ptsx2.push_back(ptsx[i]);
			ptsy2.push_back(ptsy[i]);

			//cout<<i<<" ptsx:"<<ptsx2[i]<<" ptsy:"<<ptsy2[i]<<endl;
		}

		tk::spline s;
		s.set_points(ptsx2, ptsy2);


		for(int i=0;i<previous_path_x.size();++i){
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);

			//cout<<"0 "<<i<<" next_x_vals:"<<next_x_vals[i]<<" next_x_vals:"<<next_y_vals[i]<<endl;
		}
		
		double target_x = 30.;
		double target_y = s(target_x);
		double target_dist = sqrt(pow(target_x,2.) + pow(target_y,2.));
		double x_add_on = 0.;

		for(int i=1; i<= 50-previous_path_x.size(); ++i){
			double N = target_dist / (0.02 * ref_vel / 2.24);
			double x_point = x_add_on + target_x/N;
			double y_point = s(x_point);

			double x_ref = x_point;
			double y_ref = y_point;

			x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
			y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

			x_point += ref_x;
			y_point += ref_y;

			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);	
			//cout<<"1 "<<i<<" next_x_vals:"<<next_x_vals[i]<<" next_x_vals:"<<next_y_vals[i]<<endl;
			
		}*/	

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
