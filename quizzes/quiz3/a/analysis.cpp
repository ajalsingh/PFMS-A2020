#include "analysis.h"

#include <iostream>
#include <cmath>

Analysis::Analysis()
{

}

void Analysis::setShapes(std::vector<Shape*> shapes) {
    shapes_=shapes;
}

void Analysis::setLine(Line line) {
    line_=line;
}

std::vector<bool> Analysis::intersectsLine(){
    for (auto s:shapes_){
        radius_.push_back(sqrt(s->getArea()/M_PI)); //can't use getRadius method from Circle class (not sure why), so radius is calculated using area
    }
    std::vector<double> eqn;    //dx,dy,dr,D
    eqn = line_.getEquations();
    double discrim;
    for (int i= 0; i<shapes_.size();i++){
        discrim = pow(radius_.at(i),2)*eqn.at(2) - pow(eqn.at(3),2);
        if (discrim >= 0){
            intersect_.push_back(1);
        }
        else{
            intersect_.push_back(0);
        }
    }
    return intersect_;
}

//! TODO - TASK 3: Implement the missing function(s) in Analysis Class
//!
//! HINT: Analysis inherits from AnalysisInterface which is an abstract class
//! What type of class is AnalysisInterface, what does this mean?
//!
//! Use the following webiste to assist in developing the code
//! https://mathworld.wolfram.com/Circle-LineIntersection.html
//!
//! BONUS QUESTION (FOR DISCUSSION)
//! At the moment we are implementing check intersect for a Circle
//! as the only question used.
//! If we had a Rectangle shape, how could we differentiate between the approach
//! to intersect for the Rectangle and the Circle?
