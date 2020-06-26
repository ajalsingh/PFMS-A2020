#include <iostream>
#include "rectangle.h"
#include "circle.h"


int main () {
    //! TODO: Create a rectangle
    Rectangle rectangle;
    rectangle.setHeightWidth(5,3.5);

    // Print some info about it
    std::cout << "The area of box is " << rectangle.getArea() << std::endl;
    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    Circle circle(3);
    std::cout << "The area of circle is " << circle.getArea() << std::endl;
    std::cout << "It is a " << circle.getDescription() << std::endl;

    circle.setCentre(3,2);
    std::cout << "The x coord is " << circle.getCentreX() <<  "The y coord is " << circle.getCentreY() <<std::endl;
    std::cout << "intersects? " << circle.checkIntercept(2.5,3)<<std::endl;
    //!ADDITIONAL QUESTIONS TO CONSIDER
    // - Would it make sense to create an instance of shape? No
    // - A design to prohibit this is covered when we introduce polymorphism

}
