#include "rectanglehelper.h"
#include <iostream>

RectangleHelper::RectangleHelper(double width,double height)
{
  setHeightWidth(width,height);
}

void RectangleHelper::printArea(){

  std::cout << getDescription() <<  " " ;
  std::cout << "area=[" <<  getArea() << "]" << std::endl;

}
