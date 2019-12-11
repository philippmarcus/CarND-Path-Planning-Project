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
};

class Path {
    public:
        vector<Point> pts;
        vector<double> getX();
        vector<double> getY();
        Point get(int id);
        void push_back(Point p);
        int size();
};

struct CarState {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

vector<Point> GeneratePath(CarState car, vector<Point> previous_path, double desired_lane, double desired_speed);

#endif  // PATHPLANNING_H