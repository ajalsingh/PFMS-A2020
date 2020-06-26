#include <iostream>
#include <vector>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"
#include "shape.h"

using std::cout;
using std::endl;
using std::vector;

int main () {

    vector<Shape*> shapes;
    shapes.push_back(new Triangle(3.0, 4.0));
    shapes.push_back(new Triangle(1.0, 5.0));
    shapes.push_back(new Rectangle(2.0, 3.0));
    shapes.push_back(new Rectangle(6.0, 4.0));
    shapes.push_back(new Circle(3.0));
    shapes.push_back(new Circle(4.0));
    shapes.push_back(new Circle(5.0));

    for (auto s : shapes) {
        cout << s->getDescription() << " has area " <<  (*s).getArea() << endl;
    }
}
