#include <iostream>
#include <thread>
#include <chrono>
#include "radar.h"

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

ms duration;

void queryData(Radar &radar);

int main (void){

  //! TASK 1:
  //! Instantiate radar object
  //! Display the default max distance
  std::shared_ptr<Radar> radarPointer(new Radar());
  auto radar = (*radarPointer);
  std::cout<<"Default Max Distance: "<<radar.getMaxDistance()<<std::endl;

  //! TASK 2
  //! Create a thread tied to the spawn member function of Radar [radar.h](./a/dep/radar.h).
  //! Create a while loop in the main runs 50 times and displays the return value of getData member funcion.
  
  std::thread radar_thread(&Radar::spawn, radarPointer);

  auto start_time = Time::now();
  queryData(radar);
  auto end_time = Time::now();

  //! TASK 3: We were not provided a rate for the getData
  //! Using the chrono library and 50 sucsessive calls to getData in the while loop
  //! of your main you have already developed, compute the refresh rate (running rate)
  //! of getData (This will tell us the scanning time of the sensor).

  duration = std::chrono::duration_cast<ms>((end_time - start_time)/50);
  std::cout<<"\nRefresh Rate: "<<duration.count()<<"ms\n"<<std::endl;

  //! TASK 4: The scanning time is dependent on the MaxDistance.
  //! Add to your main code that sets the other supported MaxDistance and another
  //! while loop that queries getData another 50 times.
  //! Using the chrono library and these 50 sucsessive calls to getData in the while loop,
  //! compute the refresh rate (running rate) of getData (This will tell us the scanning time
  //! of the sensor in the other supported configuration).
  radar.setMaxDistance(160);
  std::cout<<"Max Distance: "<<radar.getMaxDistance()<<std::endl;

  start_time = Time::now();
  queryData(radar);
  end_time = Time::now();

  duration = std::chrono::duration_cast<ms>((end_time - start_time)/50);
  std::cout<<"\nRefresh Rate: "<<duration.count()<<"ms"<<std::endl;

  radar_thread.join();
  return 0;
}

void queryData(Radar &radar){
  unsigned int count = 0;

  while (count < 50){
    std::vector<double> data = radar.getData();
    std::cout<<count+1<<") ";
    for (auto val : data)
    {
      std::cout << val << " ";
    }
    std::cout << std::endl;
    count++;
  }
}