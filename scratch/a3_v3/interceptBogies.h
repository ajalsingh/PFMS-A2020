/**
 * @defgroup Integration
 * @file    interceptBogies.h
 * @ingroup Integration
 * @class   InterceptBogies
 * @brief   This class integrates (via thread safe functions) incoming sensor data, performs calculations of said data into
 *          useful types, and then sends it to the friendlyControl class. 
 * @note    This class requires includes from simulator, friendlyData, and friendlyControl Classes.
 * @author  Ajal Singh
 * @version 1.0
 * @date    May 2020
 */

#ifndef INTERCEPT_BOGIE_H
#define INTERCEPT_BOGIE_H

//_____________________________________________________________________________ Includes

#include "simulator.h"
#include "friendlyData.h"
#include "friendlyControl.h"
#include "processData.h"

#include <vector>
#include <mutex>
#include <condition_variable>

//_____________________________________________________________________________ Struct Definitions
/**
 * @brief Thread safe container for vector of RangeBearingStamped
 * 
 */
struct BogieRBS{
    std::vector<RangeBearingStamped> rbs;
    std::mutex mu;
    std::condition_variable cv;
};

/**
 * @brief Thread safe container for Pose
 * 
 */
struct poseStruct
{
    Pose pose;
    std::mutex mu;
    std::condition_variable cv;
};

//_____________________________________________________________________________ Class Definition

class InterceptBogies{
//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new Intercept Bogies object
     * 
     * @param sim shared pointer to object of Simulator
     */
    InterceptBogies(const std::shared_ptr<Simulator> & sim);

    /**
     * @brief Destroy the Intercept Bogies object
     * 
     */
    ~InterceptBogies();

    /**
     * @brief Thread to control friendly aircraft
     * 
     */
    void controlThread();

    /**
     * @brief Thread to scan for data
     * 
     */
    void scanThread();

    /**
     * @brief Thread to process scanned data
     * 
     */
    void processThread();

//_____________________________________________________________________________ Class Private Members
private:
//_____________________________________________________________________________ Private Member Functions
    /**
     * @brief Converts scannned data (RangeBearingStamped) into BogieCraft for compatability with control class
     *      \n Sorts bogies by range from friendly, creates a bogie instance and fills params.
     * 
     * @param raw_rbs Raw RangeBearingStamped data from scan class
     * @param friend_pose current pose of friendly aircraft
     */
    void determineBogieParams(std::vector<RangeBearingStamped> &raw_rbs, const Pose& friend_pose);

     

//_____________________________________________________________________________ Private Member Variables
    std::shared_ptr<Simulator> sim_;                //!<    Shared pointer to Simulator class
    std::mutex mu_;                                 //!<    Global mutex used to prevent simulatenous access by threads 
    std::condition_variable cv_;                    //!<    Global convar used to synchronise threads
    bool process_ready_;                            //!<    True when scanned data is recieved and read for processing


    FriendlyData f_data_;                           //!<    Instance of FriendlyData class
    FriendlyControl f_control_;                     //!<    Instance of FriendlyControl class
    ProcessData p_data_;                            //!<    Instance of ProcessData class

    BogieRBS raw_rbs_ ;                             //!<    sorted RangeBearingStamped data (Thread Safe)
    poseStruct friendly_pose_;                      //!<    friendly Pose data (Thread safe)
    BogieCraftContainer bogies_container_;          //!<    Container for vector of BogieCraft (Thread safe)
    std::vector<Pose> previous_bogie_;              //!<    bogie positions from previous scan
};
#endif //INTERCEPT_BOGIE_H