#ifndef _VEHICLE_H_
#define _VEHICLE_H_
#include "helpers.h"
#include <string>
class Vehicle{
	public:
	enum VehicleState{
		vsKL,
		vsPLCL,
		vsPLCR,
		vsLCL,
		vsLCR
	};
	
	Vehicle(double s = 0., double d = 0., double yaw = 0., double speed = 0., double x = 0., double y = 0.);
	
	void update(double s, double d, double yaw, double speed, double x, double y);
	
	void process(const std::vector<double>& map_waypoints_x,
  			     const std::vector<double>& map_waypoints_y,
  				 const std::vector<double>& map_waypoints_s,
				 const std::vector<double>& previous_path_x,
				 const std::vector<double>& previous_path_y,
				 const std::vector<std::vector<double> >& sensor_fusion
				);
	

	
	std::vector<double> ptsx() const;
	std::vector<double> ptsy() const;
	
private:
	std::string state_name(VehicleState state) const;
	int lane_from_d() const;
	int lane_from_d(double d) const;

	void process_trajectory(const std::vector<double>& map_waypoints_x,
  			     			const std::vector<double>& map_waypoints_y,
  				 			const std::vector<double>& map_waypoints_s,
				 			const std::vector<double>& previous_path_x,
				 			const std::vector<double>& previous_path_y,
						   	int lane,
						    double speed);
	
	enum VehiclePosition{
		vpAhead,
		vpBehind
	};
	
	bool find_vehicle(Vehicle& vehicle, VehiclePosition check_position, const std::vector<std::vector<double> >& sensor_fusion, double dist = 30., int lane = -1) const;
	
	std::vector<VehicleState> successor_states() const;
	
	VehicleState next_state(VehicleState current) const;

	
	VehicleState best_state(const std::vector<VehicleState>& states, const std::vector<std::vector<double> >& sensor_fusion) const;
	//for devbug purpose
	void process2(const std::vector<double>& map_waypoints_x,
  			     const std::vector<double>& map_waypoints_y,
  				 const std::vector<double>& map_waypoints_s,
				 const std::vector<double>& previous_path_x,
				 const std::vector<double>& previous_path_y);
	
	void pts_to_vehicle_coords(const std::vector<double>& origin_xy, double yaw);
	void pts_to_global_coords(const std::vector<double>& origin_xy, double yaw);
	
	void pts_to_vehicle_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy);
	void pts_to_global_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy);
	
	std::vector<double> m_ptsx;
	std::vector<double> m_ptsy;
	double m_s, m_d, m_yaw, m_speed, m_x, m_y;
	int m_target_lane;
	double m_target_speed;
	
	VehicleState m_state;
	
};


#endif