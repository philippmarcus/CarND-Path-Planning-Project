
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
//#include "helpers.h"
#include "spline.h"
#include "pathplanning.h"
#include "map.h"

// for convenience
using std::string;
using std::vector;

Point Point::rotate(double angle) {
    double rotated_x = x * cos(0 - angle) - y * sin(0 - angle);
    double rotated_y = x * sin(0 - angle) + y * cos(0 - angle);

    Point result{rotated_x, rotated_y};
    return result;
}
Point Point::add(Point shift_vector) {
    double shifted_x = x + shift_vector.x;
    double shifted_y = y + shift_vector.y;

    Point result{shifted_x, shifted_y};
    return result;
}
Point Point::subtract(Point shift_vector) {
    double shifted_x = x - shift_vector.x;
    double shifted_y = y - shift_vector.y;

    Point result{shifted_x, shifted_y};
    return result;
}
double Point::distance(Point p) {
    double result = sqrt((x -p.x)*(x-p.x)+(y-p.y)*(y-p.y));
    return result;
}

vector<double> Path::getX() {
    vector<double> result;
    for(std::vector<Point>::size_type i = 0; i != pts.size(); ++i){
        result.push_back(pts[i].x);
    }
    return result;
}
vector<double> Path::getY(){
    vector<double> result;
    for(std::vector<Point>::size_type i = 0; i != pts.size(); ++i){
        result.push_back(pts[i].y);
    }
    return result;
}
Point Path::get(int id){
    return pts[id];
}
void Path::push_back(Point p){
    pts.push_back(p);
}
int Path::size() {
    return pts.size();
}

Path GeneratePath(CarState car, vector<Point> previous_path,
                            double desired_lane, double desired_speed, Map map) {

    Path result_path;

    int prev_path_size = previous_path.size();

    // add all unconsumed former points to the path
    for(int i = 0; i < prev_path_size; ++i) {
        result_path.push_back(previous_path[i]);
    }

    ///////////////////////////////
    ///// generate a spline
    ///////////////////////////////

    // convert anchor points to car-based reference system
    Path anchor_pts;

    // The spline should be tangent to the last part of the path. Generate 2 points!
    if(prev_path_size > 1) {
        // Use the last two points
        anchor_pts.push_back(previous_path[prev_path_size-2]);
        anchor_pts.push_back(previous_path[prev_path_size-1]);
    } else {
        // We need to make one point up in the past to get a yaw angle    

        Point cur =     {car.x, car.y};
        Point prev =    {car.x - cos(car.yaw),
                         car.y - sin(car.yaw)};

        anchor_pts.push_back(prev);
        anchor_pts.push_back(cur);
    }
    
    // Additionally create 3 new anchor points further ahead of the car
    double new_d = desired_lane * 4. + 2.;
    vector<double> wp1 = map.getXY(car.s + 30, new_d);
    vector<double> wp2 = map.getXY(car.s + 60, new_d);
    vector<double> wp3 = map.getXY(car.s + 90, new_d);
    anchor_pts.push_back({wp1[0], wp1[1]});
    anchor_pts.push_back({wp2[0], wp2[1]});
    anchor_pts.push_back({wp3[0], wp3[1]});

    // Reference for spline generation is the first of the anchor points
    Point ref = anchor_pts.get(1);
    double ref_yaw = atan2(anchor_pts.get(1).y - anchor_pts.get(0).y, anchor_pts.get(1).x - anchor_pts.get(0).x);

    // Anchor points shifted and rotated
    Path transf_anchor_pts;
    for(std::vector<Point>::size_type i = 0; i != anchor_pts.size(); i++){
        // shift points by current x,y vector of the car
        Point result = anchor_pts.get(i).subtract(ref).rotate(0 - ref_yaw);
        transf_anchor_pts.push_back(result);
    }
    
    // set spline points
    tk::spline s;
    s.set_points(transf_anchor_pts.getX(), transf_anchor_pts.getY());

    // Get target waypoint and compute distance
    double d = transf_anchor_pts.get(1).distance(transf_anchor_pts.get(4));

    int N = 50 - prev_path_size;
    const double CONST_MPH_TO_MPS = 0.44704;

    double ref_vel = car.speed;

    for(int i = 0; i < N; ++i) {

        // double x_val = transf_anchor_ptsx[1] + (i+1) * step_size;
        double x_val = transf_anchor_pts.get(1).x + (i+1) * 0.02 * ref_vel * CONST_MPH_TO_MPS;
        double y_val = s(x_val);
        
        // Choose the increment, so that given the number of steps N, the car gets as close as possible to the desired target speed
        if (ref_vel < 49.5) {
            ref_vel += 0.12;
        }

        Point p{x_val, y_val};
        Point result = p.rotate(ref_yaw).add(ref);

        // rotate points by current angle of the car and add to result
        result_path.push_back(result);
    }
    return result_path;
}