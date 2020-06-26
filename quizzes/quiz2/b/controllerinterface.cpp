#include "controllerinterface.h"
#include <chrono>

ControllerInterface::ControllerInterface()
{}

void ControllerInterface::startTimer(){
    start_timer_ = std::chrono::high_resolution_clock::now();
    time_taken_ = std::chrono::duration<double>::zero();
}

void ControllerInterface::stopTimer(){
    auto stop_timer = std::chrono::high_resolution_clock::now();
    time_taken_ = stop_timer - start_timer_;
}

int ControllerInterface::getTime(){
    return std::chrono::duration_cast<std::chrono::milliseconds>(time_taken_).count();
}
