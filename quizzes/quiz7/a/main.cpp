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

  // Bugatti - Veyron
  // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg


  // Toyota - Yaris_WRsC
  // height = 1.19 m, width = 1.87 m, power P = 420 HP, drag coefficient = 0.30, weight = 1190 kg
  Car Merc("Mercedes", "C180", 1.45, 1.77, 143, 0.29, 1200);
  Car Bugatti("Bugatti", "Veyron", 1.19, 2.00, 1200, 0.35, 2200);
  Car Toyota("Toyota", "Yaris_WRsC", 1.19, 1.87, 420, 0.3, 1190);

  // Add the cars to a vector of Cars

  std::vector<Car> cars = {Merc, Bugatti, Toyota};
  std::vector<bool> reached_top_speed;

  //! @todo
  //! TASK 2
  //! Write a loop that uses the 'Cars' and prints out the make, model and top speed
  //!
  //! CONSIDER: If you have 3 seperate cars you will not be able to loop over them
  for (auto it=cars.begin();it!=cars.end();it++){
    Car car = *it;
    std::cout<<"Make: "<<car.getMake()<<std::endl;
    std::cout<<"Model: "<<car.getModel()<<std::endl;
    std::cout<<"Top Speed: "<<car.getTopSpeed()<<std::endl;
    std::cout<<std::endl;

    reached_top_speed.push_back(false);
  }


  bool still_racing = true;

  //! @todo
  //! TASK 3
  //!
  //! Race the vehicles by:
  //! 1. Accelerating each vehicle until it reachs top speed
  //! 2. When each vehicle reaches top speed deccelerate it
  //! 3. When the first vehicle reaches zero speed then stop the race

  DisplayRace raceDisplay; // This creates a OpenCV window to display the race

  //! Race until time lapsed is less than duration or reaching top speed
  while (still_racing){
    for (unsigned int i=0; i<cars.size(); i++){
      if (reached_top_speed[i]){
        cars.at(i).decelerate();
        if (cars.at(i).getCurrentSpeed() <= 0){
          still_racing = false;
        }
      }
      else{
        cars.at(i).accelerate();
        if (cars.at(i).getCurrentSpeed() >= cars.at(i).getTopSpeed()){
          reached_top_speed[i] = true;
        }
      }
    }
    //Uncomments the below once you have a vector of Car called cars
    raceDisplay.updateDisplay(cars);
    
    //Slow down the thread for 50 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
    return 0;
}
