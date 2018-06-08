#ifndef _VEHICLE_H_
#define _VEHICLE_H_
#include "helpers.h"

class Vehicle{
	public:
	Vehicle(double s, double d, double yaw, double speed, double x, double y);
	
	void process(const std::vector<double>& map_waypoints_x,
  			     const std::vector<double>& map_waypoints_y,
  				 const std::vector<double>& map_waypoints_s,
				 const std::vector<double>& previous_path_x,
				 const std::vector<double>& previous_path_y);
	void process2(const std::vector<double>& map_waypoints_x,
  			     const std::vector<double>& map_waypoints_y,
  				 const std::vector<double>& map_waypoints_s,
				 const std::vector<double>& previous_path_x,
				 const std::vector<double>& previous_path_y);
	std::vector<double> ptsx() const;
	std::vector<double> ptsy() const;
	
private:
	void pts_to_vehicle_coords(const std::vector<double>& origin_xy, double yaw);
	void pts_to_global_coords(const std::vector<double>& origin_xy, double yaw);
	
	void pts_to_vehicle_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy);
	void pts_to_global_coords(const std::vector<double>& origin_xy, double yaw, std::vector<double>& ptsx, std::vector<double>& ptsy);
	
	std::vector<double> m_ptsx;
	std::vector<double> m_ptsy;
	double m_s, m_d, m_yaw, m_speed, m_x, m_y;
	
};


#endif