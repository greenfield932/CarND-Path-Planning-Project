#ifndef _HELPERS_H_
#define _HELPERS_H_
#include <math.h>
#include <vector>
namespace sf_fields{//sensor fusion fields
	const int kId = 0;
	const int kX = 1;
	const int kY = 2;
	const int kVx = 3;
	const int kVy = 4;
	const int kS = 5;
	const int kD = 6;
}

namespace track_info{
	const double kSpeedLimit = 50.;
	const double kLaneWidth = 4.;
	const int kLaneCount = 3;
}

namespace vehicle_info{
	const double kMaxAccelS = 10;//m/s^2
	const double kMaxAccelD = 10;//m/s^2
}

// For converting back and forth between radians and degrees.
constexpr double pi(){ 
	return M_PI; 
}

double distance(double x1, double y1, double x2, double y2);

double deg2rad(double x);

double rad2deg(double x);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);

double mph2mps(double mph);//miles per hour to meters per second
double mps2mph(double mps);//meters per second to miles per hour
	

#endif