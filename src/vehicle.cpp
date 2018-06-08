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
	m_y(y)
{
	
}



void Vehicle::process(const vector<double>& map_waypoints_x,
  			          const vector<double>& map_waypoints_y,
  				     const vector<double>& map_waypoints_s,
					 const vector<double>& previous_path_x,
					 const vector<double>& previous_path_y){
	
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
	
	int lane = (int(m_s)/100)%3;
	
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

	
	m_ptsx.clear();
	m_ptsy.clear();
	//double ref_vel = 15;
	//double target_x = 30.;
	//double target_y = s(target_x);
	//double target_dist = sqrt(pow(target_x,2.) + pow(target_y,2.));
	
	double target_dist = 30.;
	double ref_vel = mph2mps(49.5);
	double dt = 0.02;
	
	double dist_step = ref_vel*dt;
	
	vector<double> ptsx, ptsy;

	for(int i=1; i <= 50 - previous_path_x.size(); ++i){
		double x_point = i*dist_step;		
		double y_point = s(x_point);
		ptsx.push_back(x_point);
		ptsy.push_back(y_point);	
	}
	
	pts_to_global_coords({ref_x, ref_y}, ref_yaw, ptsx, ptsy);
	
	m_ptsx = previous_path_x;
	m_ptsy = previous_path_y;
	
	for(int i=0;i<ptsx.size();++i){
		m_ptsx.push_back(ptsx[i]);
		m_ptsy.push_back(ptsy[i]);
	}	
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

vector<double> Vehicle::ptsx() const{
	return m_ptsx;
}

vector<double> Vehicle::ptsy() const{
	return m_ptsy;
}