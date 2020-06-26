#include "circle.h"
#include <cmath>

Circle::Circle(double radius): radius_(radius)
{
    description_="circle";
}

void Circle::setRadius(double radius)
{
    radius_ = radius;
    description_="circle";

}

double Circle::getArea(void)
{
    return 2*M_PI*radius_;
}

bool Circle::checkIntercept(double x, double y){
    if (((x - radius_) >= 0 ) || (y - radius_ >= 0)){
        return false;
    }
    else{
        return true;
    }
}
