#include "processData.h"

ProcessData::ProcessData(){
    previous_velocities_ = {220,220};
}

ProcessData::~ProcessData(){
    
}

double ProcessData::distance(const GlobalOrd& p1, const GlobalOrd& p2){
    double diff_x = fabs(p2.x - p1.x);
    double diff_y = fabs(p2.y - p1.y);
    return sqrt(pow(diff_x,2) + pow(diff_y,2));
}

double ProcessData::calcOrientation(const Pose &current_pose, const Pose &previous_pose){
    //find the x and y distances between poses
    double x = current_pose.position.x - previous_pose.position.x;
    double y = current_pose.position.y - previous_pose.position.y;
    
    //Find the heading 
    double heading = atan2(y,x);

    //adjust heading from 0-2 Pi
    if (heading < 0){
        return heading += 2*M_PI;
    }
    else{
        return heading;
    }
}

Pose ProcessData::predictBogiePose(const Pose &current_pose, const Pose &previous_pose, const double& seconds_in_future){
    Pose future_bogie;
    double avg_x, avg_y;

    //Find velocity in x and y direction
    double velocity_x = (current_pose.position.x - previous_pose.position.x)/0.110;
    double velocity_y = (current_pose.position.y - previous_pose.position.y)/0.110;

    //store previous velocity
    previous_velocities_ = {velocity_x, velocity_y};
    
    //set pose and orientation of future bogie
    future_bogie.position.x = current_pose.position.x + velocity_x*seconds_in_future;
    future_bogie.position.y = current_pose.position.y + velocity_y*seconds_in_future;
    future_bogie.orientation = current_pose.orientation;

    return future_bogie;
}

std::pair<double,double> ProcessData::getFutureRangeBearingToFriend(const Pose&current_bogie, const Pose& future_bogie, const Pose& friendly){
    
    //calculate distances between all three poses
    double range_diff = this->distance(current_bogie.position, future_bogie.position);
    double friend_to_current = this->distance(current_bogie.position, friendly.position);
    double friend_to_future = this->distance(future_bogie.position, friendly.position);
    
    //calculate bearing to future pose from friendly
    double bearing = atan((future_bogie.position.x - friendly.position.x)/(future_bogie.position.y - friendly.position.y));
    
    //transform bearing from 0 - 2 Pi
    if (current_bogie.position.x > future_bogie.position.x){
        bearing = -bearing;
    }

    return {bearing ,friend_to_future};

}

AircraftState ProcessData::checkState(const Pose& pose){
    // check if aircraft is inside airspace
    const int airspace_limit = 4000 - 300;
    if (pose.position.x > airspace_limit || pose.position.x < -airspace_limit ||
        pose.position.y > airspace_limit || pose.position.y < -airspace_limit){
            return BS_UNKNOWN;
        }
    else{
        return BS_STRAIGHT;
    }
}