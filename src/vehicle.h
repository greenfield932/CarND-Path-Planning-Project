#ifndef _VEHICLE_H_
#define _VEHICLE_H_
#include "helpers.h"
#include <string>
class Vehicle{
	public:
	
	Vehicle(double s = 0., double d = 0., double yaw = 0., double speed = 0., double x = 0., double y = 0.);
	
	//update vehicle kinematics and position
	void update(double s, double d, double yaw, double speed, double x, double y);
	
	//process trajectory generation with respect to surrounding objects
	void process(const std::vector<double>& map_waypoints_x,
  			       const std::vector<double>& map_waypoints_y,
  				     const std::vector<double>& map_waypoints_s,
				       const std::vector<double>& previous_path_x,
				       const std::vector<double>& previous_path_y,
				       const std::vector<std::vector<double> >& sensor_fusion);
	

	
	//result trajectory points x
	std::vector<double> ptsx() const;
	
	//result trajectory points y
	std::vector<double> ptsy() const;
	
private:
	//Vehicle states
	enum VehicleState{
		vsKL,
		vsPLCL,
		vsPLCR,
		vsLCL,
		vsLCR
	};
	
	//Vehicle position to search for
	enum VehiclePosition{
		vpAhead,
		vpBehind
	};
	
	
	//convert state id to string name
	std::string state_name(VehicleState state) const;

	//calc lane number from d value
	int lane_from_d() const;
	
	//calc lane number from d value
	int lane_from_d(double d) const;
	
	//calc d value from lane number
	double lane_to_d(int lane) const;

	//process spline based trajectory for simulator
	void process_trajectory(const std::vector<double>& map_waypoints_x,
  			     			const std::vector<double>& map_waypoints_y,
  				 			const std::vector<double>& map_waypoints_s,
				 			const std::vector<double>& previous_path_x,
				 			const std::vector<double>& previous_path_y,
						   	int lane,
						    double speed,
						    double horizon = 50*vehicle_info::kDt);
	
	//find vehicle on the specified lane ahead or behind the car position
	bool find_vehicle(Vehicle& vehicle, VehiclePosition check_position, const std::vector<std::vector<double> >& sensor_fusion, double dist = vehicle_info::kObserveDist, int lane = -1) const;
	
	//get all feasible states based on current state
	std::vector<VehicleState> successor_states() const;
	
	//calculate cost for the state based on surrounding vehicles, their distance and speed
	double state_cost(VehicleState state, const std::vector<std::vector<double> >& sensor_fusion) const;
	
	//unconditional state change to next state
	VehicleState next_state(VehicleState current) const;

	//find best state based on environment and cost functions
	VehicleState best_state(const std::vector<VehicleState>& states, const std::vector<std::vector<double> >& sensor_fusion) const;

	//transform coordinates to vehicle coordinate system
	void pts_to_vehicle_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy) const;
	
	//transform coordinates to global coordinate system
	void pts_to_global_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy) const;
	
	std::vector<double> m_ptsx;//result points of trajectory, x
	std::vector<double> m_ptsy;//result points of trajectory, y
	double m_s, m_d, m_yaw, m_speed, m_x, m_y; // vehicle kinematics/position values
	int m_target_lane;//used when lane change processing
	double m_target_speed;	//variable to update car speed over frames
	VehicleState m_state;   //state of the vehicle (FSM)
};


#endif