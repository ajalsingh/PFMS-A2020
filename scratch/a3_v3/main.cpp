/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  TODO: Start and join Control, Scan and Process Threads
 *
 *  @author {TODO: Ajal Singh + 12621189}
 *  @date {27/05/20}
*/
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include "interceptBogies.h"


int main(void)
{
  std::vector<std::thread> threads;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  std::shared_ptr<InterceptBogies> IB(new InterceptBogies(std::ref(sim)));

  threads.push_back(sim->spawn());
  threads.push_back(std::thread(&InterceptBogies::controlThread, IB));  //control thread
  threads.push_back(std::thread(&InterceptBogies::scanThread, IB));
  threads.push_back(std::thread(&InterceptBogies::processThread, IB));
  //Join threads and begin!
  for(auto & t: threads){
    t.join();
  }

  return 0;
}
