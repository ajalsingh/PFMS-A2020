#ifndef CIRCLE
#define CIRCLE

#include "shape.h"

class Circle: public Shape
{
public:
    Circle(double radius);
    void setRadius(double radius);
    double getRadius (void);
    double getArea (void);
    bool checkIntercept(double x, double y);

private:
    double radius_; //radius of circle
};
#endif // CIRCLE

