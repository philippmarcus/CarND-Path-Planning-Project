#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; };
constexpr double CONST_Miles_To_KM = 1.6093;
constexpr double CONST_MPH_TO_MPS = 0.44704;
constexpr double safety_dist(double vel) {return CONST_Miles_To_KM*vel/2.0; };

double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y);
                     
#endif  // HELPERS_H