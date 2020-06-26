#ifndef RADAR
#define RADAR

#include <string>
#include <vector>
#include <chrono>

class Radar{
public:
    /* @brief   Default constructor for radar
     *          Configures all fixed parameters and assigns default variable couple params.
     */
    Radar();

    /*  @brief Destructor for radar
     */
    ~Radar();

    /* @brief Constructor for radar. Configures all fixed params.
     *
     *  @param Accepts coupled params, scanning time in milliseconds and max distance in meters
     */
    Radar(int scan_time_ms, int max_distance_metres);

    /*  @brief Returns model of string type
     */
    std::string getModel();

    /*  @brief Returns output of string type
     */
    std::string getOutput();

    /*  @brief Returns field of view of radar as int
     */
    int getFieldOfViewDeg();

    /*  @brief Returns minimum distance (metres) of radar as type double
     */
    double getMinDist();

    /*  @brief Returns maximum allowed targets of radar as type int
     */
    int getMaxTargets();

    /*  @brief Returns selected scan time (milliseconds) of radar as type int
     */
    int getScanTime();

    /* @brief Set the value of scanning time
     *
     *  @param Accepts coupled param, scanning time, in milliseconds
     */
    bool setScanTime(int scan_time_ms);

    /*  @brief Returns selected max distance (metres) value of radar as type double
     */
    double getMaxDist();

    /* @brief Set the maximum distance in metres
     *
     *  @param Accepts couple param, maximum distance in metres
     */
    bool setMaxDist(double max_distance_metres);

    /*  @brief Returns a vector data doubles containing targets, range and bearing
     */
    std::vector<double> getData();

    /*  @brief Set random generated data doubles in vector. Includes targets, range and bearing
     */
    void setData();

    /*  @brief Polls data for new values
     */
    double poll();

    /*  @brief Returns time taken since last configuration change
     */
    double getConfigTime();

    /*  @brief Returns time taken since first query of data
     */
    double getQueryTime();

protected:
    /*  @brief Set random number of targets
     */
    int setTargets();

    /*  @brief Returns numbre of targets
     */
    int getTargets();

    /*  @brief Starts timer after configuration change
     */
    void configTimerStart();

    /*  @brief Stops configuration change timer
     */
    void configTimerStop();

    /*  @brief Starts timer after first query of data
     */
    void queryTimerStart();

    /*  @brief Stops query change timer
     */
    void queryTimerStop();

    /*  @brief Returns the data sample number
     */
    int getSampleCount();

    /*  @brief reset sample number
     */
    void resetSampleCount();

private:
    std::string model_;             /*  Model of radar */
    std::string output_;            /*  Type of output of radar */
    int field_of_view_degrees_;     /*  Field of view of radar */
    double min_distance_metres_;    /*  Minimum scan distance of radar */
    int max_number_of_targets_;     /*  Maximum number of targets supported by radar */
    int scanning_time_;             /*  Scanning time of radar */
    int max_distance_;              /*  Maximum allowed scan distance of radar */

    int number_of_targets_;         /*  Number of targets detected by radar */
    double distance_;               /*  Stores the random distances generated before being passed to vector (vec_range_bearing_) */
    int bearing_;                   /*  Stores the random bearings generated before being passed to vector (vec_range_bearing_) */

    int sample_count_;                          /*  Stores current sample number */

    /*  @brief  Stores all target data.
     *          First element stores the number of targets.
     *          Seconds element stores the current sample number
     *          The third and fourth elements respectively store the range and bearing data of the first target
     *          The fifth and sixth elements respectively store the range and bearing data of the second target
     *          This pattern continues, hence storing range and bearing data for each of the targets.
     */
    std::vector<double> vec_range_bearing_;


    std::chrono::high_resolution_clock::time_point start_config_time_point_;    /*  Captures the start time-point of configuration timer */
    std::chrono::duration<double> time_since_config_change_;                    /*  The time since last cofig change */
    std::chrono::high_resolution_clock::time_point start_query_time_point_;     /*  Captures the start time-point of first query timer */
    std::chrono::duration<double> time_since_first_query_;                      /*  Time since first query for data */
};

#endif // RADAR

