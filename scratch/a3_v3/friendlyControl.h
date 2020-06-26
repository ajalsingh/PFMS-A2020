/**
 * @defgroup Control
 * @file    friendlyControl.h
 * @ingroup Control
 * @class   FriendlyControl
 * @brief   This class calculates and returns linear and angular velocities
 *          when fed a pose of vector of BogiCraft type.
 *          \n This class also supports the simulator and allows velocity adjustment directly.
 *          \n This class requires the PurePursuit class to destroy bogies
 * @note    This class requires includes from purePursuit and Simulator (if required).
 * @author  Ajal Singh
 * @version 1.0
 * @date    May 2020
 */

//_____________________________________________________________________________ Includes
#ifndef FRIENDLYCONTROL_H
#define FRIENDLYCONTROL_H
#include "simulator.h"
#include "purePursuit.h"

//_____________________________________________________________________________ Class Definition
class FriendlyControl{
//_____________________________________________________________________________ Class Public Members
public:

    /**
     * @brief Construct a new Friendly Control object
     * 
     */
    FriendlyControl();

    /**
     * @brief Construct a new Friendly Control object with simulator functionality
     * 
     * @param sim shared pointer instance of Simulator
     */
    FriendlyControl(const std::shared_ptr<Simulator>& sim);

    /**
     * @brief Destroy the Friendly Control object
     * 
     */
    ~FriendlyControl();

    /**
     * @brief Set the Simulator object in order to use adjustVelocity private method
     * 
     * @param sim shared pointer instance of Simulator 
     */
    void setSimulator(const std::shared_ptr<Simulator>& sim);

    /**
     * @brief Keeps aircraft inside the airspace 
     * 
     * @param current_friendly_pose Current Pose of aircraft
     * @return std::pair<double, double> = {linear velocity, Angular velocity}
     */
    std::pair<double, double> keepInsideAirspace(const Pose &current_friendly_pose);

    /**
     * @brief Aims the friendly aircraft towards nearest bogie inside the airspace only
     * 
     * @param bogies vector of type Bogiecraft
     * @param friendly_pose current Pose of friendly 
     * @return std::pair<double, double> = {linear velocity, Angular velocity}
     */
    std::pair<double, double> destroyBogie(const std::vector<BogieCraft> & bogies, const Pose& friendly_pose);

//_____________________________________________________________________________ Class Private Members
private:
//_____________________________________________________________________________ Private Member Functions

    /**
     * @brief checks the state of given pose within the airspace
     * 
     * @param pose Pose
     * @return AircraftState  BS_UNKNOWN or BS_STRAIGHT
     */
    AircraftState checkState(const Pose &pose);

    /**
     * @brief Adjusts the linear and angular velocity of friendly
     * @note Requires the simulator class
     * @note Requires the shared pointer to 'Simulator' instance to be set by the setSimulator function 
     * @param lin linear velocity
     * @param ang angular velocity
     */
    void adjustVelocity(const double &lin, const double &ang);

    /**
     * @brief checks weather set linear velocity is within bounds.
     *          Will alter value accordingly is out of bounds
     * 
     * @param lin_vel linear velocity
     * @return double - linear velocity (adjusted if required)
     */
    double checkLinVelocity(const double &lin_vel);

    /**
     * @brief checks weather set angular velocity is within bounds.
     *          Will alter value accordingly is out of bounds
     * 
     * @param ang_vel angular velocity
     * @return double - angular velocity (adjusted if required)
     */
    double checkAngVelocity(const double &ang_vel);

//_____________________________________________________________________________ Private Member Constants
    
    const double AIRSPACE_SIZE_ = 4000;                                     //!<    The length to the bounday of airspcae from centre point (Base Station)
    const int G_ = 6;                                                       //!<    Maximum Gs permitted 
    const double GRAVITY_ = 9.81;                                           //!<    Acceleration due to gravity
    const double MAX_LIN_VELOCITY_ = 900;                                   //!<    Maximum linear velocity
    const double TERMINAL_LIN_VELOCITY_ = 50;                               //!<    Minimum linear velocity
    const double MIN_ANG_VELOCITY_ = 0;                                      //!<    Minimum angular velocity
    const double MAX_ANG_VELOCITY_ = G_*GRAVITY_/TERMINAL_LIN_VELOCITY_;    //!<    Maximum angular velocity

//_____________________________________________________________________________ Private Member Variables

    PurePursuit pp_;                        //!<    Instance of Purepursuit class 
    std::shared_ptr<Simulator> sim_;        //!<    Shared pointer to Simulator class
    bool inside_airspace_;                  //!<    True when aircraft is within airspace
    double lin_velocity_;                   //!<    Linear Velocities
    double ang_velocity_;                   //!<    Angular Velocities
};

#endif //FRIENDLYCONTROL_H