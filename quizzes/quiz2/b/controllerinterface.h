#ifndef CONTROLLERINTERFACE_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define CONTROLLERINTERFACE_H

#include <string>
#include <chrono>

// The ControllerInterface is a class which specifies the minimum
// required interface for your child classes
class ControllerInterface
{
public:
  ControllerInterface();

  /**
   This function gets the current speed
   */
  virtual double getCurrentSpeed(void) = 0;


  //Essential for driving the vehicle

  /**
   This function decelerates the vehicle
   */
  virtual void decelerate(void) = 0;

  /**
   This function accelerates the vehicle
   */
  virtual void accelerate(void) = 0;

  /**
   This function starts a timing
   */
  void startTimer();

  /**
   This function stops a timing
   */
  void stopTimer();

  /**
   This function return the time period
   */
  int getTime();

protected:
  double time_;

  std::chrono::high_resolution_clock::time_point start_timer_;
  std::chrono::duration<double> time_taken_;

};


#endif // CONTROLLERINTERFACE_H
