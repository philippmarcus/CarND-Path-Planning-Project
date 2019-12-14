#ifndef PATHPLANNING_H
#define PATHPLANNING_H


#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "map.h"

// for convenience
using std::string;
using std::vector;

class Point {
    public:
        Point(double a_x, double a_y) : x{a_x}, y{a_y} {}
        const double x;
        const double y;
        Point rotate(double angle);
        Point add(Point shift_vector);
        Point subtract(Point shift_vector);
        double distance(Point p);
        friend std::ostream& operator<<(std::ostream &os, Point const &p);
};

class Path {
    public:
        Path(vector<double> ptsx, vector<double> ptsy);
        Path();
        vector<Point> pts;
        vector<double> getX();
        vector<double> getY();
        Point get(int id);
        void push_back(Point p);
        int size();
};

class CarState {
    public:
        CarState(double a_x, double a_y, double a_s,
                 double a_d, double a_yaw, double a_speed) : 
            x{a_x}, y{a_y}, s{a_s}, d{a_d}, yaw{a_yaw}, speed{a_speed} {}
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    private:
};

Path GeneratePath(CarState car, Path previous_path,
                            double desired_lane, double desired_speed, Map map);

#endif  // PATHPLANNING_H