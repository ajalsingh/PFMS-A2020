#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape
{
public:
    Rectangle();
    void setHeightWidth(double width, double height);
    bool intersects(double x, double y);
    
    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Consider if getArea() is a method that should exist in Rectangle?
    // Answer: Each shape is unique and has a different formula to calculate area. Since calculating the area of a circle would require a different formula, getArea() should be specific to the type of shape

    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // Answer: Yes all shapes should be able to compute area.

    // A design to enable this is covered in when we introduce polymorphism
    double getArea (void);
    double checkIntercept(double x, double y);

private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?
    // Answer: The variables are specific to a rectangle. Other shapes will require other specific variables.

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
