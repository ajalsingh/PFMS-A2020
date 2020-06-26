/**
 * @defgroup Scanner
 * @file    friendlyData.h
 * @ingroup Scanner
 * @class   FriendlyData
 * @brief   This class returns scanned bogie data from onboard the onboard radar, friendly position
 *          data through a high-precision INSGPS, and has the ability to retreive data provided by the BaseStation.
 * @note    Base Station Class must be included (inheritance).
 * @author  Ajal Singh
 * @version 1.0
 * @date    May 2020
 */

#ifndef FRIENDLYDATA_H
#define FRIENDLYDATA_H

//_____________________________________________________________________________ Includes
#include "baseStation.h"

//_____________________________________________________________________________ Class Definition
class FriendlyData: public BaseStation{

//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new Friendly Data object
     * 
     */
    FriendlyData();

    /**
     * @brief Destroy the Friendly Data object
     * 
     */
    ~FriendlyData();

    /**
     * @brief Get the Data From Friendly object
     * 
     * @return std::vector<RangeBearingStamped> data from onbard radar
     */
    std::vector<RangeBearingStamped> getDataFromFriendly();

    /**
     * @brief Get the Friendly Pose object
     * 
     * @return Pose data from high-precision INSGPS
     */
    Pose getFriendlyPose();
};

#endif //FRIENDLYDATA_H