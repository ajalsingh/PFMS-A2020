// We need to include the declaration of our new rectangle class in order to use it.
#include "circle.h"
#include <iostream>
#include <vector>

double circleAreaCombined(std::vector<Circle> circles){
    double totalArea;
    for (int i=0;i<circles.size();i++){
        totalArea += circles.at(i).area();
    }
    return totalArea;
}

double randomCircles(){

}

int main () {
    //Create a Circle object
    //Circle c1(1.0), c2(2.0), c3(5.0);
    std::vector<Circle> circles;

    circles.push_back(Circle(1.0));
    circles.push_back(Circle(2.0));
    circles.push_back(Circle(5.0));

    double area=0, perimeter=0;
    for (auto circle : circles){
        std::cout << "Area: "<<circle.area()<<" Perim: "<< circle.perimeter() << std::endl;
        area +=circle.area();
        perimeter +=circle.perimeter();
    }
    std::cout << "Area: "<<area<<" Perim: "<< perimeter << std::endl;

//    Circle circle[3] = {c1, c2, c3};

//    for (int x=0;x<3;x++){
//        circles.push_back(circle[x]);
//        double area = circles.at(x).area();
//        double perim =circles.at(x).perimeter();
//        std::cout << "Area: "<<area<<" Perim: "<< perim << std::endl;
//    }
    double totalArea = circleAreaCombined(circles);
    std::cout<<"Total area of circles: "<< totalArea<< std::endl;
    return 0;
}

