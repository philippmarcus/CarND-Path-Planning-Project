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
};

#endif  // MAP_H