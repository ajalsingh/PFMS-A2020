#include "analysis.h"

#include <iostream>

Analysis::Analysis()
{
}

//! TODO - TASK 3: Implement the missing function(s) in Analysis Class

void Analysis::setShapes(std::vector<Shape *> shapes)
{
    shapes_ = shapes;
};

void Analysis::setLine(Line line)
{
    line_ = line;
};




std::vector<bool> Analysis::intersectsLine()
{
   double x1, x2, y1, y2;
    std::vector<bool> check;
    
    
    return check;
};
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
