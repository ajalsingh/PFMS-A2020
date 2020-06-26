// We need to include the declaration of car class in order to use it.
#include "car.h"
// We need to include the declaration of display class in order to use it.
#include "display_race.h"

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include<vector>


int main (void) {

  //! @todo
  //! TASK 1
  //! Create 3 cars with follwing specifications
  //!
  //! CONSIDER: We will be using all the cars for a race and need to treat all of them as a collection

  // Mercedes - C180
  // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

    Car Mercedes("Mercedes", "C180", 1.45, 1.77, 143, 0.29, 1200);
    Car* car1 = &Mercedes;

  // Bugatti - Veyron
  // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg

    Car Bugatti("Bugatti", "Veyron", 1.19, 2.00, 1200, 0.35, 2200);
    Car* car2 = &Bugatti;

  // Toyota - Yaris_WRsC
  // height = 1.19 m, width = 1.87 m, power P = 420 HP, drag coefficient = 0.30, weight = 1190 kg

    Car Toyota("Toyota", "Yaris_WRsC", 1.19, 1.87, 420, 0.30, 1190);
    Car* car3 = &Toyota;

  // Add the cars to a vector of Cars

    std::vector<Car*> cars;
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

  //! @todo
  //! TASK 2
  //! Write a loop that uses the 'Cars' and prints out the make, model and top speed
  //!
  //! CONSIDER: If you have 3 seperate cars you will not be able to loop over them

    for (auto i = cars.begin(); i != cars.end(); i++){
        std::cout<<"Car Model: "<< (*i)->getModel() << std::endl;
        std::cout<<"Car Make: "<< (*i)->getMake() <<std::endl;
        std::cout<<"Top Speed: "<< (*i)->getTopSpeed() <<std::endl;
        std::cout<<"_____________________________"<<std::endl;
    }

  bool still_racing = true;

  //! @todo
  //! TASK 3
  //!
  //! Race the vehicles by:
  //! 1. Accelerating each vehicle until it reachs top speed
  //! 2. When each vehicle reaches top speed deccelerate it
  //! 3. When the first vehicle reaches zero speed then stop the race

  bool bugatti_accelerate = true;
  bool mercedes_accelerate = true;
  bool toyota_accelerate = true;
  bool winner = false;

  DisplayRace raceDisplay; // This creates a OpenCV window to display the race

  //! Race until time lapsed is less than duration or reaching top speed
  while (still_racing){

      if (bugatti_accelerate){
          Bugatti.accelerate();
          if (Bugatti.getCurrentSpeed() >= Bugatti.getTopSpeed()){
              bugatti_accelerate = false;
          }
      }

      if (mercedes_accelerate){
          Mercedes.accelerate();
          if (Mercedes.getCurrentSpeed() >= Mercedes.getTopSpeed()){
              mercedes_accelerate = false;
          }
      }

      if (toyota_accelerate){
          Toyota.accelerate();
          if (Toyota.getCurrentSpeed() >= Toyota.getTopSpeed()){
              toyota_accelerate = false;
          }
      }

      //! Decelearate after reachig top speed to zero

      if (!bugatti_accelerate){
          if (Bugatti.getCurrentSpeed() <= 0){

              if (!winner){
                  std::cout << "Bugatti finishes first! Details:" << std::endl;
                  std:: cout << Bugatti.getMake() << " " << Bugatti.getModel() << std::endl;
                  winner = true;
              }
          }
          Bugatti.decelerate();
      }

      if (!mercedes_accelerate){
          if (Mercedes.getCurrentSpeed() <= 0){

              if (!winner){
                  std::cout << "Mercedes finishes first! Details:" << std::endl;
                  std:: cout << Mercedes.getMake() << " " << Mercedes.getModel() << std::endl;
                  winner = true;
              }
          }
          Mercedes.decelerate();
      }

      if (!toyota_accelerate){
          if (Toyota.getCurrentSpeed() <= 0){

              if (!winner){
                  std::cout << "Toyota finishes first! Details:" << std::endl;
                  std:: cout << Toyota.getMake() << " " << Toyota.getModel() << std::endl;
                  winner = true;
              }
          }
          Toyota.decelerate();
      }

      if (winner){
          still_racing = false;
      }

    //Uncomments the below once you have a vector of Car called cars
    raceDisplay.updateDisplay(cars);

    //Slow down the thread for 50 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
    return 0;
}
