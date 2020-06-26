#include "circle.h"

Circle::Circle(double radius): radius_(radius){}

void Circle::setRadius(double radius){
    radius_ = radius;
}

double Circle::area(){
    double area = 3.14159*radius_*radius_;
    return area;
}

double Circle::perimeter(){
    double perim = 3.14*2*radius_;
    return perim;
}
