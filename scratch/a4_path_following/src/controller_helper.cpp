#include "controller_helper.h"

VelocityControlHelper::VelocityControlHelper(){
    
}

VelocityControlHelper::~VelocityControlHelper(){

}

bool VelocityControlHelper::checkForObstacle(const sensor_msgs::LaserScanPtr& msg){
    int ranges_size = msg->ranges.size();
    
    // check if robot is within obstacle limit and inform user
    for (int i = ranges_size/4; i < ranges_size*3/4; i++){

        // Intensities obstacle
        if (msg->intensities.at(i)){
            if (msg->ranges.at(i) <= 0.6){
                return true;
            }
        }

        // Solid wall/boundary obstacle
        else{
            if (msg->ranges.at(i) <= 0.4){
                return true;
            }
        }
    }
    return false;
}
