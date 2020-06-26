/**
 * @file    baseStation.h
 * @ingroup Scanner
 * @class   BaseStation
 * @brief   This class returns scanned bogie data from the base station
 * @author  Ajal Singh
 * @version 1.0
 * @date    May 2020
 */

#ifndef BASESTATION_H
#define BASESTATION_H

//_____________________________________________________________________________ Includes
#include "simulator.h"

//_____________________________________________________________________________ Class Definition

class BaseStation{

//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new Base Station object
     * 
     */
    BaseStation();

    /**
     * @brief Destroy the Base Station object
     * 
     */
    ~BaseStation();

    /**
     * @brief Set the Simulator object in order to use adjustVelocity private method
     * 
     * @param sim shared pointer instance of Simulator 
     */
    void setSimulator(const std::shared_ptr<Simulator>&sim);

    /**
     * @brief Get the Data From Base Station
     * 
     * @return std::vector<RangeVelocityStamped> base station data
     */
    std::vector<RangeVelocityStamped> getDataFromBase(); 

    /**
     * @brief Interface function to get data from friendly's onboard sensors
     * 
     * @return std::vector<RangeBearingStamped> 
     */
    virtual std::vector<RangeBearingStamped> getDataFromFriendly() = 0;

//_____________________________________________________________________________ Class Protected Members
protected:
    std::shared_ptr<Simulator> sim_;    //!<    Shared pointer to Simulator class
};

#endif //BASESTATION_H