// We need to include the declaration of our new car class in order to use it.
#include "car.h"

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <vector>

int main (void) {

    //Some specifications provided here, though you can use any of your own

    //Mercedes C180 Compressor.
    // height = 1.45 m, width = 1c m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

    //Bugatti Veyron Super Sport.
    // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg


    //!TODO - TASK 2 : instatiate two objects of type `Car` with different specifications and determine's their top speed.
    // Create a Bugatti and Mercedes object
    Car Bugatti("Bugatti", "Veyron Super Sport",1.19, 2.00, 1200.0, 0.35, 2200.0);
    Car Mercedes("Mercedes", "C180 Compressor",1.45, 1.45, 143.0, 0.29, 1200.0);

    std::vector<Car*> cars;
    cars.push_back(&Bugatti);
    cars.push_back(&Mercedes);

    Bugatti.calculateTopSpeed();
    Mercedes.calculateTopSpeed();
    Bugatti.setStationaryState(0);
    Mercedes.setStationaryState(0);

    bool still_racing = true;
    int speed_check = 0;
    int accelerate = true;

    //Slow down the thread for 100 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    std::cout<<"The race has started! Please wait..."<<std::endl;


    //! Race until races ends
    while (still_racing){
        for (auto car : cars){
            if (car->getCurrentSpeed() < car->calculateTopSpeed() && car->getTopSpeedReached() == 0){
                car->accelerate();
            }
            else{
                if (car->getCurrentSpeed() >= car->calculateTopSpeed()){
                    car->setTopSpeedReached(1);
                }
            }
        }

        //!TODO - TASK 4 : Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero

        //! Accelerate cars to top speed

//        for (auto itr= cars.begin(); itr!=cars.end(); itr++){
//            Car* temp = *itr;
//            if (temp->getCurrentSpeed() < temp->calculateTopSpeed() && temp->getTopSpeedReached() == 0){
//                temp->accelerate();
//                //std::cout<<temp->getMake()<<" "<<temp->getCurrentSpeed()<<std::endl;
//            }
//            else{
//                if (temp->getCurrentSpeed() >= temp->calculateTopSpeed()){
//                    temp->setTopSpeedReached(1);
//                }
//            }
//        }

        //! Decelearate after reachig top speed to zero

        accelerate = false;
        for (auto itr= cars.begin(); itr!=cars.end(); itr++){
            Car* temp = *itr;
            if (temp->getCurrentSpeed() > 0 && temp->getTopSpeedReached() == 1){
                temp->decelerate();
                //std::cout<<temp->getMake()<<" "<<temp->getCurrentSpeed()<<std::endl;
            }
        }

        //! Print when each car reach top speed

        for (auto itr= cars.begin(); itr!=cars.end(); itr++){
            Car* temp = *itr;
            if (temp->getCurrentSpeed() > temp->calculateTopSpeed()){
                for (int x=0; x<1;x++);{
                    std::cout<<temp->getMake()<< " "<<temp->getModel()<<" has reached its top speed!"<<std::endl;
                }
            }
        }

        //! Print the car details of fisrt car to reach speed of zero

        for (auto itr= cars.begin(); itr!=cars.end(); itr++){
            Car* temp = *itr;
            if (temp->getCurrentSpeed() == 0 && speed_check == 0 && temp->getStationaryState() == 0){
                temp->setStationaryState(1);
                std::cout<<temp->getMake()<< " "<<temp->getModel()<<" was first to reach 0!"<<std::endl;
                speed_check = 1;
                if (cars.size() == 2){
                    cars.erase(itr--);
                    Mercedes.startTimer();
                }
            }
        }

        //! Print the current speed of all other cars

        for (auto itr= cars.begin(); itr!=cars.end(); itr++){
            Car* temp = *itr;
            if (temp->getCurrentSpeed() != 0 && speed_check == 1){
                std::cout<<temp->getMake()<<"'s speed is currently: "<<temp->getCurrentSpeed()<<std::endl;
                speed_check++;
            }
            //! get stopping time
            if (temp->getCurrentSpeed() == 0 && temp->getStationaryState() == 0 && speed_check == 2){
                Mercedes.stopTimer();
                speed_check++;
                std::cout<<temp->getMake()<<" stopped "<<Mercedes.getTime()/1000<<" seconds later"<<std::endl;
                still_racing = false;
            }
        }


      //! NOTE: Keep the below sleep for the thread of 50 miscroseconds
      std::this_thread::sleep_for(std::chrono::microseconds(50));

      }

    return 0;
}
