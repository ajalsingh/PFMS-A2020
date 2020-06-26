// We need to include the declaration of our new car class in order to use it.
#include "car.h"

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include<vector>

int main (void) {

    //Some specifications provided here, though you can use any of your own

    //Mercedes C180 Compressor.
    // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

    //Bugatti Veyron Super Sport.
    // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg


    //!TODO - TASK 2 : instatiate two objects of type `Car` with different specifications and determine's their top speed.
    // Create a Bugatti and Mercedes object


    bool still_racing = true;

    //Slow down the thread for 100 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(200));


    //! Race until races ends
    while (still_racing){
        //!TODO - TASK 4 : Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero

        //! Accelerate cars to top speed
        //! Decelearate after reachig top speed to zero
        //! Print when each car reach top speed
        //! Print the car details of fisrt car to reach speed of zero
        //! Print the current speed of all other cars 

      //! NOTE: Keep the below sleep for the thread of 50 miscroseconds
      std::this_thread::sleep_for(std::chrono::microseconds(50));


      }
  }
    return 0;
}
