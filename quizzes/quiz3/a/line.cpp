#include "line.h"
#include <cmath>
#include <iostream>

Line::Line():
    gradient_(0.0),
    y_intercept_(0.0)
{
}

Line::Line(double gradient, double y_intercept):
    gradient_(gradient),
    y_intercept_(y_intercept)
{
}

Line::Line(double ax, double ay, double bx, double by)
{
    fromPoints(ax, ay, bx, by);
}

void Line::fromPoints(double ax, double ay, double bx, double by)
{
    gradient_ = (by - ay) / (bx - ax);
    y_intercept_ = ay - gradient_ * ax;
    setEquations(ax, ay, bx, by);
}

void Line::setGradient(double gradient)
{
    gradient_ = gradient;
}

double Line::getGradient(){
    return gradient_;
}

void Line::setYIntercept(double y_intercept)
{
    y_intercept_ = y_intercept;
}

double Line::getYIntercept(){
    return y_intercept_;
}

bool Line::pointAboveLine(double x, double y)
{
    double line_y = gradient_ * x + y_intercept_;
    return y > line_y;
}

void Line::setEquations(double ax, double ay, double bx, double by){
    double dx,dy,dr, D;
    dx = bx - ax;
    dy = by - ay;
    dr = pow(dx,2)+pow(dy, 2);
    D = ax*by - bx*ay;
    equations_.push_back(dx);
    equations_.push_back(dy);
    equations_.push_back(dr);
    equations_.push_back(D);
}

std::vector<double> Line::getEquations(){
    return equations_;
}
