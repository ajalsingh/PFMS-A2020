#include "friendlyData.h"

FriendlyData::FriendlyData(){

}

FriendlyData::~FriendlyData(){

}

std::vector<RangeBearingStamped> FriendlyData::getDataFromFriendly(){
    return (sim_->rangeBearingToBogiesFromFriendly());
}

Pose FriendlyData::getFriendlyPose(){
    return (sim_->getFriendlyPose());
}