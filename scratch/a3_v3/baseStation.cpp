#include "baseStation.h"

BaseStation::BaseStation(){

}

BaseStation::~BaseStation(){

}

void BaseStation::setSimulator(const std::shared_ptr<Simulator>& sim){
    sim_ = sim;
}

std::vector<RangeVelocityStamped> BaseStation::getDataFromBase(){
    return (sim_->rangeVelocityToBogiesFromBase());
}