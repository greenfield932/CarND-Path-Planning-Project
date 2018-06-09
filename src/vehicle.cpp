#include "vehicle.h"
#include "spline.h"
#include <iostream>
#include <iomanip>
using namespace std;


Vehicle::Vehicle(double s, double d, double yaw, double speed, double x, double y):
	m_s(s),
	m_d(d),
	m_yaw(yaw),
	m_speed(speed),
	m_x(x),
	m_y(y),
	m_state(vsKL),
	m_target_lane(-1),
	m_target_speed(track_info::kSpeedLimit)
{
	
}

void Vehicle::update(double s, double d, double yaw, double speed, double x, double y){
	m_s = s;
	m_d = d;
	m_yaw = yaw;
	m_speed = speed;
	m_x = x;
	m_y = y;
	
	m_ptsx.clear();
	m_ptsy.clear();
}

std::vector<Vehicle::VehicleState> Vehicle::successor_states() const{

	std::vector<VehicleState> states;
	states.push_back(vsKL);
	if(m_state == vsKL){
		int lane = lane_from_d();
		if(lane>0){
			states.push_back(vsPLCL);
		}
		if(lane<track_info::kLaneCount-1){
			states.push_back(vsPLCR);			
		}		
	}
	else if(m_state == vsLCL){
		
	}
	else if(m_state == vsLCR){
		
	}
	else if(m_state == vsPLCL){
		states.push_back(vsPLCL);					
		states.push_back(vsLCL);			
	}
	else if(m_state == vsPLCR){
		states.push_back(vsPLCR);					
		states.push_back(vsLCR);					

	}
	
	return std::move(states);
}

int Vehicle::lane_from_d() const{
	return lane_from_d(m_d);
}

int Vehicle::lane_from_d(double d) const{
	return int(d/track_info::kLaneWidth);
}

void Vehicle::process_trajectory(const std::vector<double>& map_waypoints_x,
  			     			const std::vector<double>& map_waypoints_y,
  				 			const std::vector<double>& map_waypoints_s,
				 			const std::vector<double>& previous_path_x,
				 			const std::vector<double>& previous_path_y,
						   	int lane,
						    double speed)
{
	int prev_size = previous_path_x.size();
	
	double ref_x = m_x;
	double ref_y = m_y;
	double ref_yaw = deg2rad(m_yaw);
	
	if(prev_size < 2){
		double prev_x = m_x - cos(ref_yaw);
		double prev_y = m_y - sin(ref_yaw); 
		m_ptsx.push_back(prev_x);
		m_ptsy.push_back(prev_y);
		m_ptsx.push_back(m_x);
		m_ptsy.push_back(m_y);
	}	
	else{
		if(prev_size>2){			
			//required to minimize jerk on lane change
			//this helps to create smooth trajectory because spline will be build taking into account current trajectory
			m_ptsx.push_back(previous_path_x[0]);
			m_ptsy.push_back(previous_path_y[0]);
		}
		
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];
		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
		m_ptsx.push_back(ref_x_prev);
		m_ptsx.push_back(ref_x);
		m_ptsy.push_back(ref_y_prev);
		m_ptsy.push_back(ref_y);
	}
	
	
	double target_lane_d = track_info::kLaneWidth/2. + track_info::kLaneWidth*lane;
	
	vector<double> next_wp0 = getXY(m_s + 50, target_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(m_s + 100, target_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(m_s + 150, target_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	m_ptsx.push_back(next_wp0[0]);
	m_ptsx.push_back(next_wp1[0]);
	m_ptsx.push_back(next_wp2[0]);

	m_ptsy.push_back(next_wp0[1]);
	m_ptsy.push_back(next_wp1[1]);
	m_ptsy.push_back(next_wp2[1]);
	
	pts_to_vehicle_coords({ref_x, ref_y}, ref_yaw);
	
	tk::spline s;
	s.set_points(m_ptsx, m_ptsy);

	
	m_ptsx.clear();
	m_ptsy.clear();
	
	//Point interval 0.02 sec
	//max speed 50 mph
	//m meters   1 sec
	//x			0.02 sec
	//max distance = 0.02*m/1
	
	double ref_vel = mph2mps(speed);
	double dt = 0.02;
	
	double dist_step = ref_vel*dt;
	
	vector<double> ptsx, ptsy;

	for(int i=1; i <= 50 - previous_path_x.size(); ++i){
		double x_point = i*dist_step;		
		double y_point = s(x_point);
		ptsx.push_back(x_point);
		ptsy.push_back(y_point);	
	}
	
	/*for(int i=1; i <= 50 - previous_path_x.size(); ++i){
		double x_point = i*dist_step;		
		double y_point = s(x_point);
		ptsx.push_back(x_point);
		ptsy.push_back(y_point);	
	}*/
	
	pts_to_global_coords({ref_x, ref_y}, ref_yaw, ptsx, ptsy);
	
	m_ptsx = previous_path_x;
	m_ptsy = previous_path_y;
	
	for(int i=0;i<ptsx.size();++i){
		m_ptsx.push_back(ptsx[i]);
		m_ptsy.push_back(ptsy[i]);
	}	
}

Vehicle::VehicleState Vehicle::next_state(VehicleState current) const{

	if(current == vsKL){
		int lane = lane_from_d();
		if(lane>0){
			return vsPLCL;
		}
		else if(lane < track_info::kLaneCount - 1){
			return vsPLCR;			
		}
	}
	else if(m_state == vsPLCL){
		return vsLCL;			
	}
	else if(m_state == vsPLCR){
		return vsLCR;
	}
	return vsKL;	
	
}

void Vehicle::process(const vector<double>& map_waypoints_x,
  			          const vector<double>& map_waypoints_y,
  				     const vector<double>& map_waypoints_s,
					 const vector<double>& previous_path_x,
					 const vector<double>& previous_path_y,
					 const std::vector<std::vector<double> >& sensor_fusion){	
	
	double max_speed = track_info::kSpeedLimit - track_info::kSpeedLimit*0.01;
	//double target_speed = max_speed;
	if(m_state == vsKL){
		//if(m_speed < max_speed){
		//	double horizon = 50*0.02;		
		//	target_speed = m_speed + vehicle_info::kMaxAccelS/2.*horizon;
		//}
		Vehicle target_vehicle;
		if(find_vehicle(target_vehicle, vpAhead, sensor_fusion)){
			//target_speed = target_vehicle.m_speed;
			//std::vector<VehicleState> states = successor_states();
			//m_state = best_state(states);
			m_state = next_state(m_state);
		}	
	}
	else if(m_state == vsPLCL || m_state == vsPLCR){
		Vehicle target_vehicle;
		int target_lane = m_state == vsPLCL ? lane_from_d() - 1 : lane_from_d() + 1;
		if(!find_vehicle(target_vehicle, vpAhead, sensor_fusion, 10, target_lane)){
			cout<<"No vehicle detected, make transition"<<endl;
			//std::vector<VehicleState> states = successor_states();
			//m_state = best_state(states);			
			//if(m_state == vsLCL || m_state == vsLCR){				
			m_target_lane = target_lane;
			m_state = next_state(m_state);
			//}
		}
		else{
			cout<<"Vehicle on line:"<<target_lane<<" is blocking meneuvor from line:"<<lane_from_d()<<endl;
		}
	}
	else if(m_state == vsLCL || m_state == vsLCR){
		if(lane_from_d() == m_target_lane){
			m_state = next_state(m_state);
			//std::vector<VehicleState> states = successor_states();
			//m_state = best_state(states);
		}
	}
	
	Vehicle target_vehicle;
	if(find_vehicle(target_vehicle, vpAhead, sensor_fusion)){
		//target_speed = target_vehicle.m_speed;		
		m_target_speed = target_vehicle.m_speed;
	}
	else{
		m_target_speed = max_speed;
	}
	
	if(m_target_speed > max_speed){
		m_target_speed = max_speed;
	}
	//double max_speed_inc_per_dt = mps2mph(vehicle_info::kMaxAccelS*0.02);
	double max_speed_inc_per_dt = 1.5;
	
	double speed_diff = m_target_speed - m_speed;
	if(fabs(speed_diff) < max_speed_inc_per_dt/2){
		max_speed_inc_per_dt = 0.;
	}
	else{
		max_speed_inc_per_dt*=speed_diff/fabs(speed_diff);
	}
	
	
	double result_speed = m_speed + max_speed_inc_per_dt;
	
	if(result_speed > max_speed){
		result_speed = max_speed;
	}
	cout<<"Target state:"<<state_name(m_state).c_str()<<endl;
	cout<<"Target speed:"<<m_target_speed<<" current speed:"<<m_speed<<" max inc:"<<max_speed_inc_per_dt<<endl;
	
	int lane = (m_state == vsLCL || m_state == vsLCR) ? m_target_lane : lane_from_d();
	//int lane = (int(m_s)/100)%3;
	process_trajectory(map_waypoints_x, map_waypoints_y, map_waypoints_s, previous_path_x, previous_path_y, lane, result_speed);
}

std::string Vehicle::state_name(Vehicle::VehicleState state) const{
	switch(state){
		case vsKL:   return "KL";
		case vsPLCL: return "PLCL";
		case vsPLCR: return "PLCR";
		case vsLCL:  return "LCL";
		case vsLCR:  return "LCR";
	}
	return "Invalid state";
}

Vehicle::VehicleState Vehicle::best_state(const std::vector<VehicleState>& states, const std::vector<std::vector<double> >& sensor_fusion) const{
	/*Vehicle target_vehicle;
	for(const auto& state : states){
		
		if(!find_vehicle(target_vehicle, vpAhead, sensor_fusion, 30, vsPLCL ? lane_from_d() - 1 : lane_from_d() + 1)){
			std::vector<VehicleState> states = successor_states();
			m_state = best_state(states);			
			if(m_state == vsLCL || m_state == vsLCR){				
				m_target_lane = vsPLCL ? lane_from_d() - 1 : lane_from_d() + 1;
			}
		}
		if(find_vehicle(target_vehicle, vpAhead, sensor_fusion)){

		}

	}*/
	return vsKL;
}

void Vehicle::pts_to_vehicle_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy){
	double ref_yaw = yaw;
	double ref_x = origin_xy[0];
	double ref_y = origin_xy[1];
	//cout<<"points to vehicle"<<" refx="<<ref_x<<" ref_y="<<ref_y<<" ref_yaw="<<ref_yaw<<endl;
	
	for(int i=0;i<m_ptsx.size();++i){
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;
		ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
	    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
		//cout<<i<<" ptsx="<<m_ptsx[i]<<" ptsy="<<m_ptsy[i]<<endl;
	}
}
void Vehicle::pts_to_global_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy){
	double ref_yaw = yaw;
	double ref_x = origin_xy[0];
	double ref_y = origin_xy[1];
	//cout<<"points to global"<<" refx="<<ref_x<<" ref_y="<<ref_y<<" ref_yaw="<<ref_yaw<<endl;
	
	for(int i=0;i<ptsx.size();++i){
		double x = ptsx[i];
		double y = ptsy[i];	
		ptsx[i] = x * cos(0. - ref_yaw) + y * sin(0. - ref_yaw) + ref_x;
		ptsy[i] = -x * sin(0. - ref_yaw) + y * cos(0. - ref_yaw) + ref_y;
		//cout<<i<<" ptsx="<<m_ptsx[i]<<" ptsy="<<m_ptsy[i]<<endl;
	}	
}


void Vehicle::process2(const vector<double>& map_waypoints_x,
  			          const vector<double>& map_waypoints_y,
  				     const vector<double>& map_waypoints_s,
					 const vector<double>& previous_path_x,
					 const vector<double>& previous_path_y){
	
	/*double dist_inc = 0.3;
	for(int i = 0; i < 50; i++){
		double next_s = m_s + (i+1)*dist_inc;
		double next_d = 6;
		vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		m_ptsx.push_back(xy[0]);
		m_ptsy.push_back(xy[1]);
	}*/
	int prev_size = previous_path_x.size();
	
	double ref_x = m_x;
	double ref_y = m_y;
	double ref_yaw = deg2rad(m_yaw);
	
	/*if(prev_size < 2){
	
		double prev_x = m_x - cos(ref_yaw);
		double prev_y = m_y - sin(ref_yaw); 
		m_ptsx.push_back(prev_x);
		m_ptsy.push_back(prev_y);
		m_ptsx.push_back(m_x);
		m_ptsy.push_back(m_y);
	}	
	else{
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];
		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		m_ptsx.push_back(ref_x_prev);
		m_ptsx.push_back(ref_x);
		m_ptsy.push_back(ref_y_prev);
		m_ptsy.push_back(ref_y);
	}*/
	
		cout<<"prev size:"<<prev_size<<endl;

	
	if(prev_size>100){
		m_ptsx = previous_path_x;
		m_ptsy = previous_path_y;
		return;
	}
	
	
	
	int lane = 1;
	
	vector<double> start_wp = getXY(m_s, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	m_ptsx.push_back(start_wp[0]);
	m_ptsy.push_back(start_wp[1]);


	
	vector<double> next_wp0 = getXY(m_s + 30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(m_s + 60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(m_s + 90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	m_ptsx.push_back(next_wp0[0]);
	m_ptsx.push_back(next_wp1[0]);
	m_ptsx.push_back(next_wp2[0]);

	m_ptsy.push_back(next_wp0[1]);
	m_ptsy.push_back(next_wp1[1]);
	m_ptsy.push_back(next_wp2[1]);
	
	pts_to_vehicle_coords({ref_x, ref_y}, ref_yaw);
	
	tk::spline s;
	s.set_points(m_ptsx, m_ptsy);
	
	//double target_x = 30.;
	//double target_y = s(target_x);
	//double target_dist = sqrt(pow(target_x,2.) + pow(target_y,2.));
	double target_dist = 50.;
	
	
	//Point interval 0.02 sec
	//max speed 50 mph
	//m meters   1 sec
	//x			0.02 sec
	//max distance = 0.02*m/1
	
	double ref_vel = mph2mps(10.5);
	cout<<"10 mph:"<<mph2mps(10.)<<endl;
	double dt = 0.02;
	
	double dist_step = ref_vel*dt;
	cout<<"Distance step=1"<<dist_step<<endl;
	
	//cout<<"ref vel:"<<ref_vel<<" N="<<N<<endl;
	
	double x_prev = 0.;
	double x = 0.;
	double y_prev = 0.;
	double y = 0.;
	
	double dist_cnt = 0;
	int cnt = 1;
	
	m_ptsx.clear();
	m_ptsy.clear();
	
	while(dist_cnt < target_dist){
		x = cnt*dist_step;
		y = s(x);
		m_ptsx.push_back(x);
		m_ptsy.push_back(y);
		double dist = distance(x, y, x_prev, y_prev);
		//cout<<"x="<<x<<" y="<<y<<" dist="<<dist<<endl;
		dist_cnt += dist;
		x_prev = x;
		y_prev = y;
		cnt++;
	}
	
	/*m_ptsx.clear();
	m_ptsy.clear();
	double ref_vel = 15;
	double target_x = 30.;
	double target_y = s(target_x);
	double target_dist = sqrt(pow(target_x,2.) + pow(target_y,2.));

	for(int i=1; i<= 50-previous_path_x.size(); ++i){
		double N = target_dist / (0.02 * ref_vel / 2.24);
		double x_point = i*target_x/N;
		//cout<<"Genx:"<<x_point<<endl;
		double y_point = s(x_point);

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
		y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		m_ptsx.push_back(x_point);
		m_ptsy.push_back(y_point);	
	}*/
	
	pts_to_global_coords({ref_x, ref_y}, ref_yaw);
	double x0 = ref_x;
	double y0 = ref_y;
	cout<<fixed;
	cout<<setprecision(2);
	cout<<setw(6);
	
	for(int i=0;i<m_ptsx.size();++i){
		//cout<<i<<" x="<<m_ptsx[i]<<"\ty="<<m_ptsy[i]<<"\tdist="<<distance(x0, y0, m_ptsx[i], m_ptsy[i])<<endl;
		x0 = m_ptsx[i];
		y0 = m_ptsy[i];
	}
	//cout<<"result points:"<<m_ptsx.size()<<endl;
	
	/*for(int i=1; i<= 50-previous_path_x.size(); ++i){
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
	}*/
}

void Vehicle::pts_to_vehicle_coords(const std::vector<double>& origin_xy, double yaw){
	//double ref_yaw = deg2rad(yaw);
	double ref_yaw = yaw;
	double ref_x = origin_xy[0];
	double ref_y = origin_xy[1];
	//cout<<"points to vehicle"<<" refx="<<ref_x<<" ref_y="<<ref_y<<" ref_yaw="<<ref_yaw<<endl;
	for(int i=0;i<m_ptsx.size();++i){
		double shift_x = m_ptsx[i] - ref_x;
		double shift_y = m_ptsy[i] - ref_y;
		
		m_ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
	    m_ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
		//cout<<i<<" ptsx="<<m_ptsx[i]<<" ptsy="<<m_ptsy[i]<<endl;

	}
}

void Vehicle::pts_to_global_coords(const std::vector<double>& origin_xy, double yaw){
	//double ref_yaw = deg2rad(yaw);
	double ref_yaw = yaw;
	double ref_x = origin_xy[0];
	double ref_y = origin_xy[1];
	//cout<<"points to global"<<" refx="<<ref_x<<" ref_y="<<ref_y<<" ref_yaw="<<ref_yaw<<endl;
	for(int i=0;i<m_ptsx.size();++i){
		double x = m_ptsx[i];
		double y = m_ptsy[i];
		
		m_ptsx[i] = x * cos(0. - ref_yaw) + y * sin(0. - ref_yaw) + ref_x;
		m_ptsy[i] = -x * sin(0. - ref_yaw) + y * cos(0. - ref_yaw) + ref_y;

		//cout<<i<<" ptsx="<<m_ptsx[i]<<" ptsy="<<m_ptsy[i]<<endl;
	}	
}

bool Vehicle::find_vehicle(Vehicle& target_vehicle, VehiclePosition check_position, const vector<vector<double> >& sensor_fusion, double dist, int lane) const{
	double min_s = dist;
	bool found = false;
	for(int i=0; i < sensor_fusion.size(); ++i){
		const vector<double>& car_data = sensor_fusion[i];
		int id = car_data[sf_fields::kId];		
		double s = car_data[sf_fields::kS];
		double d = car_data[sf_fields::kD];
		double vx = car_data[sf_fields::kVx];
		double vy = car_data[sf_fields::kVy];
		double x = car_data[sf_fields::kX];
		double y = car_data[sf_fields::kY];
		double target_d = lane == -1 ? lane_from_d() : track_info::kLaneWidth/2 + lane*track_info::kLaneWidth;
		
		if(target_d != lane_from_d(d)){
			continue;
		}
		//TODO what about second loop, how to pass 0?
		double s_diff = fabs(s - m_s);
		if(s_diff < min_s && (
				(check_position == vpAhead && s > m_s) || (check_position == vpBehind && s < m_s)
		)) {
			found = true;
			min_s = s_diff;
			target_vehicle = Vehicle(s, d, 0., sqrt(vx*vx + vy*vy), x, y);			
			cout<<"found vehicle: "<<id<<" speed: "<<target_vehicle.m_speed<<" vx:"<<vx<<" vy:"<<vy<<endl;

		}		
	}
	return found;
}

vector<double> Vehicle::ptsx() const{
	return m_ptsx;
}

vector<double> Vehicle::ptsy() const{
	return m_ptsy;
}