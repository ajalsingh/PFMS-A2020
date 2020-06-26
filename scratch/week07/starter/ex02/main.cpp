#include <iostream>
#include <thread>
#include "radar.h"

int main (void){

  // instantiate radar object
  Radar radar;

  // while (true){
  //   std::vector <double> data = radar.getData();

  //   for(auto elem : data){
  //     std::cout << elem << " ";
  //   }
  //   std::cout << std::endl;
  // }
  std::shared_ptr<Radar> ptr1(new Radar);// [Constructor is Class(nh)]
  std::shared_ptr<Radar> ptr2(new Radar);
  std::shared_ptr<Radar> ptr3(new Radar);

  std::thread radar1(&Radar::start,ptr1);
  std::thread radar2(&Radar::start,ptr2);
  std::thread radar3(&Radar::start,ptr3);

  radar1.join();
  radar2.join();
  radar3.join();

  return 0;
}

