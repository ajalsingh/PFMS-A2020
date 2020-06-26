#include <iostream>
#include <vector>
#include <random>

#include "rectangle.h"
#include "triangle.h"
#include "shape.h"
#include "circle.h"

using std::cout;
using std::endl;
using std::vector;

//!TODO - TASK 4: Printing description - Completing this / think about auto
void getArea(const vector<Shape*> shapes){
    cout<<"Areas of Shapes:"<<endl;
//    for (auto itr= shapes.begin(); itr!=shapes.end(); itr++){
//        Shape* temp = *itr;
//        double x = temp->getArea();
//        cout<<x<<endl;
//    }
    for (auto shape : shapes){
        cout<<shape->getDescription()<< " has area "<<(*shape).getArea()<<endl;
    }
}


int main () {

    //!TODO - TASK 2: Create a Square and Traingle, and store both of them in a vector of type `Shape`
    Rectangle square;
    square.setHeightWidth(4.0, 4.0);

    Triangle triangle(3.0, 4.0);

    vector<Shape*> shapes;
    shapes.push_back(&square);      //pushback the address of square
    shapes.push_back(&triangle);

    Circle circle(2.0);
    shapes.push_back(&circle);

    getArea(shapes);

    //!TODO - TASK 5: Write a program that allows the user to specify number of circles and `max_radius`.
    //! Create the circles with random lengths to be capped to `max_length`.
    int circle_amount;
    int max_radius;
    cout<<"How many circles would you like?"<<endl;
    std::cin>>circle_amount;
    cout<<"Specify max radius?"<<endl;
    std::cin>>max_radius;

    srand((unsigned) time(0));
    for (int i=0;i<circle_amount;i++){
        double random = (rand() % max_radius) + 1;
        Circle *circle = new Circle(random);
        shapes.push_back(circle);
    }
    getArea(shapes);
        //circles.at(i).setRadius((rand() % max_radius) + 1)


    //Random num gen
//    std::random_device generator;
//    std::uniform_real_distribution<double> distribution(0.0001, 4.0);
//    distribution(generator);

    // solution for TASK 5
    //OPtion 2
//    shapes.push_back(new Circle(distribution(generator)));
}
