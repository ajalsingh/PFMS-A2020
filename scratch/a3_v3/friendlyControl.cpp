#include "friendlyControl.h"

FriendlyControl::FriendlyControl(){
    inside_airspace_ = true;
}

FriendlyControl::FriendlyControl(const std::shared_ptr<Simulator>& sim){
    sim_ = sim;
    inside_airspace_ = true;
}

FriendlyControl::~FriendlyControl(){

}

void FriendlyControl::setSimulator(const std::shared_ptr<Simulator>& sim){
    sim_ = sim;
}

std::pair<double, double> FriendlyControl::keepInsideAirspace(const Pose &current_friendly_pose){
    double currentOrientation = current_friendly_pose.orientation;
    double lowerLim, upperLim, angular_vel;

    //  Determine the bounce back bearing of friendly with percentage error of 10%
    //  Sets upper and lower bounce back limits
    if (currentOrientation > M_PI){
        lowerLim = currentOrientation-M_PI - M_PI*0.1;
        upperLim = currentOrientation-M_PI + M_PI*0.1;
    }
    else{
        lowerLim = currentOrientation+M_PI - M_PI*0.1;
        upperLim = currentOrientation+M_PI + M_PI*0.1;
    }

    //  Check if current friendly position is within airspace
    if (current_friendly_pose.position.x >= AIRSPACE_SIZE_ - 300 || current_friendly_pose.position.x <= -AIRSPACE_SIZE_ + 300 || current_friendly_pose.position.y >= AIRSPACE_SIZE_ - 300 || current_friendly_pose.position.y <= -AIRSPACE_SIZE_ + 300){
        inside_airspace_ = false;
    }
    else{
        inside_airspace_ = true;
    }

    //  While not inside airspace, set angular velocity to positive or negative depending on heading prior to reaching airspace limits
    while (!(inside_airspace_)){
        if (current_friendly_pose.position.x >= AIRSPACE_SIZE_ - 300 && currentOrientation >= 1.5*M_PI ||
            current_friendly_pose.position.x <= -AIRSPACE_SIZE_ + 300 && currentOrientation <= M_PI ||
            current_friendly_pose.position.y >= AIRSPACE_SIZE_ - 300 && (currentOrientation <= 0.5*M_PI || currentOrientation >= 1.5*M_PI) ||
            current_friendly_pose.position.y <= -AIRSPACE_SIZE_ + 300 && (currentOrientation <= 1.5*M_PI || currentOrientation >= 1.9*M_PI)){
            ang_velocity_ = -MAX_ANG_VELOCITY_;
        }
        else if (current_friendly_pose.position.x >= AIRSPACE_SIZE_ - 300 && currentOrientation < 0.5*M_PI ||
                current_friendly_pose.position.x <= -AIRSPACE_SIZE_ + 300 && currentOrientation > M_PI ||
                current_friendly_pose.position.y >= AIRSPACE_SIZE_ - 300 && (currentOrientation > 0.5*M_PI || currentOrientation <= 1.5*M_PI) ||
                current_friendly_pose.position.y <= -AIRSPACE_SIZE_ + 300 && (currentOrientation > 1.5*M_PI || currentOrientation <= 0.5*M_PI)){
            ang_velocity_ = MAX_ANG_VELOCITY_;
        }

        //  
        if (sim_->getFriendlyPose().orientation > upperLim || sim_->getFriendlyPose().orientation < lowerLim){
            this->adjustVelocity(TERMINAL_LIN_VELOCITY_, ang_velocity_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else{
            inside_airspace_ = true;
        }
    }
    
    // return {this->checkLinVelocity(lin_velocity_), this->checkAngVelocity(ang_velocity_)};
}

std::pair<double, double> FriendlyControl::destroyBogie(const std::vector<BogieCraft> & bogies, const Pose& friendly_pose){
    // iterate through vector to find the nearest bogie that is inside the airspace
    for (unsigned int i=0;i<bogies.size();i++){
        if (bogies.at(i).state != BS_UNKNOWN){

            //set linear and angular velocities according to bearing of target from the friendly reference frame
            if (bogies.at(i).bearingFromFriendly > 0.5 && bogies.at(i).bearingFromFriendly <= M_PI){
                lin_velocity_ = TERMINAL_LIN_VELOCITY_;
                ang_velocity_ = MAX_ANG_VELOCITY_;
            }
            else if(bogies.at(i).bearingFromFriendly < 2*M_PI - 0.5 && bogies.at(i).bearingFromFriendly > M_PI){
                lin_velocity_ = TERMINAL_LIN_VELOCITY_;
                ang_velocity_ = -MAX_ANG_VELOCITY_;
            }
            else if(bogies.at(i).bearingFromFriendly < 0.1 || bogies.at(i).bearingFromFriendly > 2*M_PI - 0.1 || bogies.at(i).rangeFromFriendly > 3000){
                lin_velocity_ = MAX_LIN_VELOCITY_;
                ang_velocity_ = MIN_ANG_VELOCITY_;
            }

            // If none of the conditions above are met, perform pure pursuit
            else{
                pp_.doPurePursuit(friendly_pose, bogies.at(i).FuturePose);
                double gamma = pp_.getGamma();
                
                /*  Pure pursuit theory states:
                    v = w/gamma .....(1)

                    Aircraft limits state:
                    v = G*GRAVITY/w...(2)

                    Using simultaneous equations we can determine that:
                          _______________
                    w = _/G*GRAVITY*gamma
                    */
                   
                ang_velocity_ = sqrt(G_*GRAVITY_*fabs(gamma));
                if (gamma < 0){
                    ang_velocity_ = -ang_velocity_;
                }
                lin_velocity_ = ang_velocity_/gamma;
            }
            break;
        }
    }

    //return lina and angular velocities
    return {this->checkLinVelocity(lin_velocity_), this->checkAngVelocity(ang_velocity_)};    
}

AircraftState FriendlyControl::checkState(const Pose &pose){
    // airspace limits are enforced 300m before AIRSPACE_SIZE_ (4000) to prevent overshoot
    if (pose.position.x > AIRSPACE_SIZE_ - 300 || pose.position.x < -AIRSPACE_SIZE_ + 300){
        return BS_UNKNOWN;
    }
    else if (pose.position.y > AIRSPACE_SIZE_ - 300 || pose.position.y < -AIRSPACE_SIZE_ + 300){
        return BS_UNKNOWN;
    }
    else{
        return BS_STRAIGHT;
    }
}

void FriendlyControl::adjustVelocity(const double & lin, const double & ang){
    sim_->controlFriendly(lin, ang);
}

double FriendlyControl::checkLinVelocity(const double &lin_vel){
    //keep lin velocity within allowable ranges
    if (lin_vel > MAX_LIN_VELOCITY_){
        return MAX_LIN_VELOCITY_;
    }
    else if (lin_vel < TERMINAL_LIN_VELOCITY_){
        return TERMINAL_LIN_VELOCITY_;
    }
    else {
        return lin_vel;
    }
}

double FriendlyControl::checkAngVelocity(const double &ang_vel){
    //keep ang velocity within allowable range
    if (ang_vel > MAX_ANG_VELOCITY_){
        return MAX_ANG_VELOCITY_;
    }
    else if(ang_vel < -MAX_ANG_VELOCITY_){
        return -MAX_ANG_VELOCITY_;
    }
    else{
        return ang_vel;
    }
}