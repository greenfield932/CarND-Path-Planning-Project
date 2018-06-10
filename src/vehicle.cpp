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
	m_target_speed(track_info::kSpeedLimit) {
	
}

void Vehicle::update(double s, double d, double yaw, double speed, double x, double y) {
	
	m_s = s;
	m_d = d;
	m_yaw = yaw;
	m_speed = speed;
	m_x = x;
	m_y = y;
	m_ptsx.clear();
	m_ptsy.clear();
}

std::vector<Vehicle::VehicleState> Vehicle::successor_states() const {
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

int Vehicle::lane_from_d() const {
	return lane_from_d(m_d);
}

int Vehicle::lane_from_d(double d) const {
	return int(d/track_info::kLaneWidth);
}

double Vehicle::lane_to_d(int lane) const {
	return track_info::kLaneWidth/2. + track_info::kLaneWidth*lane;
}


void Vehicle::process_trajectory(const std::vector<double>& map_waypoints_x,
  			     			const std::vector<double>& map_waypoints_y,
  				 			const std::vector<double>& map_waypoints_s,
				 			const std::vector<double>& previous_path_x,
				 			const std::vector<double>& previous_path_y,
						   	int lane,
						    double speed,
							double horizon) {
	int prev_size = previous_path_x.size();
	
	double ref_x = m_x;
	double ref_y = m_y;
	double ref_yaw = deg2rad(m_yaw);
	vector<double> ptsx, ptsy;
	
	m_ptsx.clear();
	m_ptsy.clear();
	//1. Create reference points for spline
	
	//Add first reference points, current if no previous trajectory exists, otherwise first 3 points of previous trajectory
	//This required to make transition between update smooth, otherwise the car will be jerking
	if(prev_size < 3){
		double prev_x = m_x - cos(ref_yaw);
		double prev_y = m_y - sin(ref_yaw); 
		ptsx.push_back(prev_x);
		ptsy.push_back(prev_y);
		ptsx.push_back(m_x);
		ptsy.push_back(m_y);
	}	
	else{
		//required to minimize jerk on lane change
		//this helps to create smooth trajectory because spline will be build taking into account current trajectory
		ptsx.push_back(previous_path_x[0]);
		ptsy.push_back(previous_path_y[0]);

		double ref_x_prev = previous_path_x[1];
		double ref_y_prev = previous_path_y[1];
		
		ref_x = previous_path_x[2];
		ref_y = previous_path_y[2];
				
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
		
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}
	
	double target_lane_d = track_info::kLaneWidth/2. + track_info::kLaneWidth*lane;
	
	//Add a few points based on interpolation of the track position (from map waypoints)
	vector<double> next_wp0 = getXY(m_s + 30, target_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(m_s + 60, target_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(m_s + 90, target_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);
	
	//Translate all points to car coordinate system
	pts_to_vehicle_coords({ref_x, ref_y}, ref_yaw, ptsx, ptsy);
	
	//create spline
	tk::spline s;
	s.set_points(ptsx, ptsy);
	ptsx.clear();
	ptsy.clear();
	
	double ref_vel = mph2mps(speed);
	double dt = vehicle_info::kDt;	
	double dist_step = ref_vel*dt;//calculate distance step for target velocity
	int point_count = horizon/dt;
	
	//2. Generate trajectory based on spline for future horizon
	for(int i=1; i <= point_count - 3; ++i){
		double x_point = i*dist_step;		
		double y_point = s(x_point);
		ptsx.push_back(x_point);
		ptsy.push_back(y_point);	
	}
	
	//Convert points to global coordinate system
	pts_to_global_coords({ref_x, ref_y}, ref_yaw, ptsx, ptsy);
	
	//add first 3 points from previous path to make smooth transition between previous and new path
	for(int i=0;i<3 && i < previous_path_x.size();++i){
		m_ptsx.push_back(previous_path_x[i]);
		m_ptsy.push_back(previous_path_y[i]);
	}
	
	//add all generated points from spline
	for(int i=0;i<ptsx.size();++i){
		m_ptsx.push_back(ptsx[i]);
		m_ptsy.push_back(ptsy[i]);
	}
}

//unconditional next state switch
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

	//max speed = speed limit -1%
	double max_speed = track_info::kSpeedLimit - track_info::kSpeedLimit*0.01;
	
	//assume keep lane initial state
	if(m_state == vsKL){
		//move with max speed until a vehicle appears ahead
		Vehicle target_vehicle;
		if(find_vehicle(target_vehicle, vpAhead, sensor_fusion, vehicle_info::kObserveDist)){
			//if vehicle ahead present find next best state from feasible states
			std::vector<VehicleState> states = successor_states();
			m_state = best_state(states, sensor_fusion);
		}	
	}
	else if(m_state == vsPLCL || m_state == vsPLCR){		
		Vehicle target_vehicle;
		int target_lane = m_state == vsPLCL ? lane_from_d() - 1 : lane_from_d() + 1;
		//check if there are vehicles on target line
		if(!find_vehicle(target_vehicle, vpAhead, sensor_fusion, vehicle_info::kObserveDistOnLaneChange, target_lane) && 
		   !find_vehicle(target_vehicle, vpBehind, sensor_fusion, vehicle_info::kObserveDistOnLaneChange, target_lane)){
			//if no vehicles found on target lane switch to change lane state
			m_target_lane = target_lane;
			m_state = next_state(m_state);
		}
		else{
			//otherwise switch back to keep lane change in case better choice will appear
			m_state = vsKL;
		}
	}
	else if(m_state == vsLCL || m_state == vsLCR){
		//keep change lane state until the car in the lane center
		if(fabs(m_d - lane_to_d(m_target_lane)) < track_info::kLaneWidth/5){
			m_state = next_state(m_state);
		}
	}
	
	//Slowdown if vehicle ahead goes slower than our car
	Vehicle target_vehicle;
	if(find_vehicle(target_vehicle, vpAhead, sensor_fusion, vehicle_info::kDistToVehicleToSlowDown)){
		m_target_speed = target_vehicle.m_speed - target_vehicle.m_speed*0.1;
	}
	else{
		m_target_speed = max_speed;
	}
	
	if(m_target_speed > max_speed){
		m_target_speed = max_speed;
	}	
	
	//try to smoothly increase/decrese speed to target speed
	double max_speed_inc_per_dt = mps2mph(vehicle_info::kMaxAccelS*vehicle_info::kDt * 3);
	double speed_diff = m_target_speed - m_speed;
	if(fabs(speed_diff) < max_speed_inc_per_dt/2){
		max_speed_inc_per_dt = 0.;
	}
	else{
		max_speed_inc_per_dt*=speed_diff/fabs(speed_diff);
	}
	
	double result_speed = m_speed + max_speed_inc_per_dt;
	
	//limit target speed by speed limit
	if(result_speed > max_speed){
		result_speed = max_speed;
	}
	//cout<<"Target state:"<<state_name(m_state).c_str()<<endl;
	//cout<<"Target speed:"<<m_target_speed<<" current speed:"<<m_speed<<" max inc:"<<max_speed_inc_per_dt<<endl;
	
	int lane = (m_state == vsLCL || m_state == vsLCR) ? m_target_lane : lane_from_d();
	
	//create spline based trajectory with target speed and lane
	process_trajectory(map_waypoints_x, map_waypoints_y, map_waypoints_s, previous_path_x, previous_path_y, lane, result_speed);
}

//0...1
//calculate cost of distance between the nearest vehicle on the same lane
//the higher the distance the lower the cost
double distance_to_vehicle_cost(double distance_to_vehicle, double max_dist){
	return 1. - distance_to_vehicle/max_dist;
}

//0 ... 1
//calculate cost of vehicle speed in front of the car
//the higher the speed the lower cost
double vehicle_speed(double vehicle_speed){
	if(vehicle_speed > track_info::kSpeedLimit){
		return 0.;
	}	
	return (track_info::kSpeedLimit - vehicle_speed)/track_info::kSpeedLimit;
}

//calculate cost for the state based on surrounding vehicles, their distance and speed
double Vehicle::state_cost(Vehicle::VehicleState state, const std::vector<std::vector<double> >& sensor_fusion) const{
	double cost = 0.;
	double speed_weight = 1.;//speed more important when lane change than dstance to vehicle
	double distance_weight = 0.9;
	
	if(state == vsKL){
		Vehicle target_vehicle;
		if(find_vehicle(target_vehicle, vpAhead, sensor_fusion, vehicle_info::kObserveDist, lane_from_d())){
			cost += speed_weight*vehicle_speed(target_vehicle.m_speed);
			cost += distance_weight*distance_to_vehicle_cost(fabs(m_s - target_vehicle.m_s), vehicle_info::kObserveDist);
		}
	}
	else if(state == vsPLCL || state == vsPLCR){
		Vehicle target_vehicle;
		int target_lane = state == vsPLCL ? lane_from_d() - 1 : lane_from_d() + 1;
		
		double cost_ahead = 0.;
		if(find_vehicle(target_vehicle, vpAhead, sensor_fusion, vehicle_info::kObserveDist*1.5, target_lane)){			
			cost_ahead += speed_weight*vehicle_speed(target_vehicle.m_speed);
			cost_ahead += distance_weight*distance_to_vehicle_cost(fabs(m_s - target_vehicle.m_s), vehicle_info::kObserveDist*1.5);
		}
		double cost_behind = 0.;

		if(find_vehicle(target_vehicle, vpAhead, sensor_fusion, vehicle_info::kObserveDist/2., target_lane)){			
			cost_behind += speed_weight*vehicle_speed(target_vehicle.m_speed);
			cost_behind += distance_weight*distance_to_vehicle_cost(fabs(m_s - target_vehicle.m_s), vehicle_info::kObserveDist/2.);
		}
		cost = max(cost_behind, cost_ahead);
	}
	return cost;
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

//return best state from feasible states beased on cost of each state and sensor fusion data
Vehicle::VehicleState Vehicle::best_state(const std::vector<VehicleState>& states, const std::vector<std::vector<double> >& sensor_fusion) const{
	VehicleState result_state = vsKL;
	float min_cost = 100.;
	for(const auto& state : states){
		float cost = state_cost(state, sensor_fusion);
		//cout<<"State "<<state_name(state)<<" cost:"<<cost<<endl;
		if(cost < min_cost){
			min_cost = cost;
			result_state = state;
		}
	}
	return result_state;
}

//transform coordinates to vehicle coordinate system
void Vehicle::pts_to_vehicle_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy){
	double ref_yaw = yaw;
	double ref_x = origin_xy[0];
	double ref_y = origin_xy[1];	
	for(int i=0;i<ptsx.size();++i){
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;
		ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
	    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
	}
}

//transform coordinates to global coordinate system
void Vehicle::pts_to_global_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy){
	double ref_yaw = yaw;
	double ref_x = origin_xy[0];
	double ref_y = origin_xy[1];	
	for(int i=0;i<ptsx.size();++i){
		double x = ptsx[i];
		double y = ptsy[i];	
		ptsx[i] = x * cos(0. - ref_yaw) + y * sin(0. - ref_yaw) + ref_x;
		ptsy[i] = -x * sin(0. - ref_yaw) + y * cos(0. - ref_yaw) + ref_y;
	}	
}

//find vehicle on the specified lane ahead or behind the car position
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
		int target_lane =  lane == -1 ? lane_from_d() : lane;
		
		if(target_lane != lane_from_d(d)){
			continue;
		}
		//TODO what about second loop, how to pass 0?
		double s_diff = fabs(s - m_s);
		if(s_diff < min_s && (
				(check_position == vpAhead && s > m_s) || (check_position == vpBehind && s < m_s)
		)) {
			found = true;
			min_s = s_diff;
			target_vehicle = Vehicle(s, d, 0., mps2mph(sqrt(vx*vx + vy*vy)), x, y);			
			//cout<<"found vehicle: "<<id<<" lane:"<<target_lane<<" dist:"<<s_diff<<" speed: "<<target_vehicle.m_speed<<endl;
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