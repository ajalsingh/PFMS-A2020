#include "shape.h"

Shape::Shape():
    description_("unknown shape")
{
}

void Shape::setCentre(double x, double y)
{
    centreX_=x;
    centreY_=y;
}

double Shape::getCentreX()
{
    return centreX_;
}

double Shape::getCentreY()
{
    return centreY_;
}

std::string Shape::getDescription()
{
    return description_;
}
