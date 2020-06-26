/**
 * @file    ogmap_visualiser.h
 * @ingroup Visualisation
 * @class   OGMapVisual
 * @brief   Visualises goal path on an OGMap image 
 * @author  Ajal Singh
 * @version 1.0
 * @date    June 2020
 */

#ifndef OGMAP_VISUALISER_H
#define OGMAP_VISUALISER_H

//_____________________________________________________________________________ Includes
#include "ros/ros.h"

// Messages
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <thread>
#include <deque>
#include <mutex>
#include <atomic>

#include "ref_frame_conversion.h"

//_____________________________________________________________________________ Global Variables
namespace enc = sensor_msgs::image_encodings;

//_____________________________________________________________________________ Class Definition
class OGMapVisual{

//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new OGMapVisual object
     * 
     * @param nh nodehandle
     */
    OGMapVisual(ros::NodeHandle nh);

    /**
     * @brief Destroy the OGMapVisual object
     * 
     */
    ~OGMapVisual();

    /**
     * @brief Retrieves messages from /robot_0/path topic
     * 
     * @param msg 
     */
    void pathCallback(const geometry_msgs::PoseArrayPtr &msg);

    /**
     * @brief Retrieves messages from /robot_0/completed_path topic
     * 
     * @param msg 
     */
    void completedpathCallback(const geometry_msgs::PoseArrayPtr &msg);

    /**
     * @brief Retrieves messages from /robot_0/odom topic
     * 
     * @param msg 
     */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    /**
     * @brief Retrieves messages from /map_image/full topic
     * 
     * @param msg 
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief   Draws points on the map representing current goal(green), future goals(red), and completed path(blue)
     *          Publishes the image with points to /map_image/path_following
     * 
     */
    void update();

//_____________________________________________________________________________ Class Private Members
private:

//_____________________________________________________________________________ Private Member Variables
    ros::NodeHandle nh_;                                            //!< nodehandle

    ros::Subscriber path_sub_;                                      //!< Subscriber to /robot_0/path topic
    ros::Subscriber compl_path_sub_;                                //!< Subscriber to /robot_0/completed_path topic
    ros::Subscriber odom_sub_;                                      //!< Subscriber to /robot_0/odom topic
    image_transport::Subscriber map_sub_;                           //!< Subscriber to /map_image/full topic
    cv_bridge::CvImagePtr cvPtr_;                                   //!< Pointer to convert ROS images to openCV images

    image_transport::Publisher map_image_pub_;                      //!< Publisher to /map_image/path_following topic

    RefFrameConversion refframeconversion_;                         //!< Instance of RefFrameConversion

    /**
     * @brief Buffer to store PoseArrarys and vehicle pose
     * 
     */
    struct PoseArrayBuffer
    {
        std::mutex mtx;                                             //!< mutex to lock data
        std::atomic<bool> received;                                 //!< bool to indicate data to be sent
        geometry_msgs::PoseArray path;                              //!< series of poses as PoseArray
        geometry_msgs::Pose vehPose;                                //!< the vehicle pose
    };
    PoseArrayBuffer checkpoints_buffer_;                            //!< threadsafe buffer used to store poseArrays
    PoseArrayBuffer completed_checkpoints_buffer_;                  //!< threadsafe buffer used to store completed poses

    /**
     * @brief Buffer to stor image data
     * 
     */
    struct ImageDataBuffer
    {
        std::deque<std::pair<cv::Mat, ros::Time>> imageStampedDeq;  //!< deque of stamped opencv images
        std::mutex mtx;                                             //!< mutex to lock data
    };
    ImageDataBuffer raw_image_buffer_;                              //!< threadsafe buffer used to store incoming images
};

#endif //OGMAP_VISUALISER_H