
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
    double rotated_x = x * cos(angle) - y * sin(angle);
    double rotated_y = x * sin(angle) + y * cos(angle);

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
//friend std::ostream& Point::operator<<(std::ostream &os, Point const &p) { 
//    return os << "Point(x=" << p.x << ", y=" << p.y <<")";
//}

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

Path::Path(vector<double> ptsx, vector<double> ptsy) {

    assert(ptsx.size() == ptsy.size());

    for(vector<double>::size_type i = 0; i < ptsx.size(); ++i ) {
        Point p = {ptsx[i], ptsy[i]};
        pts.push_back(p);
    }
}

Path::Path() {}

Path GeneratePath(CarState car, Path previous_path, double end_s,
                            double desired_lane, double desired_speed, Map map) {

    Path result_path;

    // add all unconsumed former points to the path
    for(int i = 0; i < previous_path.size(); ++i) {
        result_path.push_back(previous_path.get(i));
        //std::cout << "resusing point (" << previous_path.get(i).x << ", " << previous_path.get(i).y << ")" << std::endl;
    }
    int prev_path_size = result_path.size();
    //std::cout << "resuding " << prev_path_size << " previous waypoints" << std::endl;

    ///////////////////////////////
    ///// generate a spline
    ///////////////////////////////

    // convert anchor points to car-based reference system
    Path anchor_pts;

    double v_spline = 0.0;
    double s_spline = 0.0;

    // The spline should be tangent to the last part of the path. Generate 2 points!
    if(prev_path_size > 1) {
        // Use the last two points
        anchor_pts.push_back(previous_path.get(prev_path_size-2));
        anchor_pts.push_back(previous_path.get(prev_path_size-1));

        v_spline = previous_path.get(prev_path_size-2).distance(previous_path.get(prev_path_size-1))/0.02;
        s_spline = end_s;
    } else {
        // We need to make one point up in the past to get a yaw angle    

        Point cur =     {car.x, car.y};
        Point prev =    {car.x - cos(car.yaw),
                         car.y - sin(car.yaw)};

        anchor_pts.push_back(prev);
        anchor_pts.push_back(cur);

        v_spline = car.speed;
        s_spline = car.s;
    }
    
    // Additionally create 3 new anchor points further ahead of the car
    double new_d = desired_lane * 4. + 2.;

    //double spline_start = car.s;//end_s ? prev_path_size > 1 : car.s;

    vector<double> wp1 = map.getXY(s_spline + 35, new_d);
    vector<double> wp2 = map.getXY(s_spline + 60, new_d);
    vector<double> wp3 = map.getXY(s_spline + 90, new_d);
    anchor_pts.push_back({wp1[0], wp1[1]});
    anchor_pts.push_back({wp2[0], wp2[1]});
    anchor_pts.push_back({wp3[0], wp3[1]});

    // Reference for spline generation is the first of the anchor points
    Point ref = anchor_pts.get(1);
    double ref_yaw = atan2(anchor_pts.get(1).y - anchor_pts.get(0).y, anchor_pts.get(1).x - anchor_pts.get(0).x);
    //std::cout << "ref_yaw " << ref_yaw << std::endl;
    // Anchor points shifted and rotated
    Path transf_anchor_pts;
    for(std::vector<Point>::size_type i = 0; i != anchor_pts.size(); i++){
        // shift points by current x,y vector of the car
        Point result = anchor_pts.get(i).subtract(ref).rotate(0 - ref_yaw);
        transf_anchor_pts.push_back(result);

        //std::cout << "anchor point (" << anchor_pts.get(i).x << ", " << anchor_pts.get(i).y 
        //            << ") is transformed to (" << transf_anchor_pts.get(i).x << ", " << transf_anchor_pts.get(i).y << ")" << std::endl;
    }
    
    // set spline points
    tk::spline s;
    s.set_points(transf_anchor_pts.getX(), transf_anchor_pts.getY());

    Point target_point = {30.0, s(30.0)};
    double x_add_on = 0.0;

    // Get target waypoint and compute distance
    double d = transf_anchor_pts.get(1).distance(target_point);

    //int N = 50 - prev_path_size;
    const double CONST_MPH_TO_MPS = 0.44704;
    //std::cout << "Creating " << N << " new points" << std::endl;

    double delta_t = 0.02;

    for(int i = 0; i < 50 - previous_path.size(); ++i) {

        double N = d/(delta_t * desired_speed * CONST_MPH_TO_MPS);

        //double x_val = transf_anchor_pts.get(1).x + (i+1) *  delta_t * desired_speed * CONST_MPH_TO_MPS;
        double x_val = x_add_on + target_point.x/N;
        double y_val = s(x_val);
        x_add_on = x_val;
        //std::cout << "New point (" << x_val << ", " << y_val << ")" << std::endl;

        Point p{x_val, y_val};
        Point result = p.rotate(ref_yaw).add(ref);

        // rotate points by current angle of the car and add to result
        result_path.push_back(result);
    }
    return result_path;
}