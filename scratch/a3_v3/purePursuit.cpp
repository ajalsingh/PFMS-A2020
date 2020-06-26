#include "purePursuit.h"

PurePursuit::PurePursuit(){

}

PurePursuit::~PurePursuit(){

}

double PurePursuit::getGamma(){
    return gamma_;
}

void PurePursuit::doPurePursuit(const Pose & current_pose, const Pose &goal_pose){
    /*  The distance from point to point can be broken down into its x and y lengths to form a triangle
        The x length and y length are calculated before determining the hypotnuse (distance from point to point)
        */
    double diff_x = goal_pose.position.x - current_pose.position.x;
    double diff_y = goal_pose.position.y - current_pose.position.y;
    double goal_range = sqrt(pow(diff_x,2) + pow(diff_y,2));

    // calculate gamma where diff_x is the look-ahead distance, and goal range is distance between the two
    gamma_ = 2*diff_x/pow(goal_range, 2);
}