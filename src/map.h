#ifndef MAP_H
#define MAP_H

#include <vector>
using std::vector;
class Map {
    public:
        Map();
        vector<double> getFrenet(double x, double y, double theta);
        vector<double> getXY(double s, double d);
        
    private:
        vector<double> maps_x;
        vector<double> maps_y;
        vector<double> maps_s;
        vector<double> map_waypoints_dx;
        vector<double> map_waypoints_dy;
        
        // Calculate closest waypoint to current x, y position
        int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);

        // Returns next waypoint of the closest waypoint
        int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                        const vector<double> &maps_y);
};

#endif  // MAP_H