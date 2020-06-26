/**
 * @defgroup DataProcessing
 * @file    processData.h
 * @ingroup DataProcessing
 * @class   ProcessData
 * @brief   This class performs calculations and returns the result.
 * @note    This class requires includes from types Class.
 * @author  Ajal Singh
 * @version 1.0
 * @date    May 2020
 */

#ifndef PROCESS_DATA_H
#define PROCESS_DATA_H

//_____________________________________________________________________________ Includes

#include "types.h"

//_____________________________________________________________________________ Class Definition
class ProcessData{
//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new Process Data object
     * 
     */
    ProcessData();


    /**
     * @brief Destroy the Process Data object
     * 
     */
    ~ProcessData();

    /**
     * @brief Calculate the distance between two points
     * 
     * @param p1 point 1 - ie. current point
     * @param p2 point 2 - ie. future point
     * @return double 
     */
    double distance(const GlobalOrd& p1, const GlobalOrd& p2);

    /**
     * @brief Determines the Heading/orientation of aircraft within the global space
     * 
     * @param current_pose current pose
     * @param previous_pose previous pose
     * @return double - Heading/orientation
     */
    double calcOrientation(const Pose &current_pose, const Pose &previous_pose);

    /**
     * @brief Predicts the bogie position in the immediate future
     * 
     * @param current_pose  current/starting pose
     * @param previous_pose     pose prior to the current from previous scan  
     * @param seconds_in_future     prediction time in futre (seconds)
     * @return Pose     Future pose of bogie
     */
    Pose predictBogiePose(const Pose &current_pose, const Pose &previous_pose, const double& seconds_in_future);

    /**
     * @brief Get the Future Range and Bearing To Friend object
     * 
     * @param current_bogie current bogie position and orientation
     * @param future_bogie  futuer bogie position and orientation
     * @param friendly  friendly aircraft pose and orientation
     * @return std::pair<double,double> = {Range from friendly, bearing from friendly (0-2 pi)}
     */
    std::pair<double,double> getFutureRangeBearingToFriend(const Pose&current_bogie, const Pose& future_bogie, const Pose& friendly);
    
    /**
     * @brief Checks whether the aircraft is within the airspace
     * 
     * @param pose current pose of aircraft
     * @return AircraftState - predefined in "types.h"
     */
    AircraftState checkState(const Pose&pose);

//_____________________________________________________________________________ Class Private Members
private:
    std::pair<double,double> previous_velocities_;  //!<< bogie velocities from previous scan
};
#endif //PROCESS_DATA_H