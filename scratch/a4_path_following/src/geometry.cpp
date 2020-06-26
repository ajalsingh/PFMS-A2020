#include "geometry.h"

Geometry::Geometry(){

}

Geometry::~Geometry(){

}

std::pair<double, double> Geometry::getDistToGoal(const double p1_x, const double p1_y, const double p2_x, const double p2_y ){
    // Subtract current point from goal point
    return {(p2_x - p1_x), (p2_y - p1_y)};
}

double Geometry::getAngleToGoal(const double dist_x, const double dist_y, const double start_yaw){
    double angle_to_goalpoint = atan2(dist_y, dist_x) - start_yaw;

    return this->normaliseAngle(angle_to_goalpoint);
}

double Geometry::getAngleDiffGoalYaw(const double goal_yaw, const double start_yaw){
    double angle_to_goalyaw = goal_yaw - start_yaw;
                
    return this->normaliseAngle(angle_to_goalyaw);
}

double Geometry::normaliseAngle(double angle){

    // Normalise angle readings within -180 and 180 degrees
    if (angle > M_PI){
        angle -= 2*M_PI; 
    }
    else if (angle < -M_PI){
        angle += 2*M_PI;
    }

    return angle;
}
   