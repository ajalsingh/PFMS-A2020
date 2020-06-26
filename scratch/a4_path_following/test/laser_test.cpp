#include <gtest/gtest.h>
#include <climits>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/LaserScan.h>

#include "../src/controller_helper.h" //This is the path of the header file in our project

TEST(Laser, checkNoObstacle){
    VelocityControlHelper vch;

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("a4_path_following");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    // The file is called smiley_og.png
    std::string file = path + "laser_no_obstacle.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScanPtr laserScan;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
        if (laserScan != nullptr){
            // Now we have a laserScan so we can proceed
            break;
        }
    }

    bag.close();

    ASSERT_NE(laserScan, nullptr);

    bool obstacle_detected = vch.checkForObstacle(laserScan);

    ASSERT_FALSE(obstacle_detected);

}

TEST(Laser, checkObstacle){
    VelocityControlHelper vch;

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("a4_path_following");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    // The file is called smiley_og.png
    std::string file = path + "laser_obstacle.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScanPtr laserScan;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
        if (laserScan != nullptr){
            // Now we have a laserScan so we can proceed
            break;
        }
    }

    bag.close();

    ASSERT_NE(laserScan, nullptr);

    bool obstacle_detected = vch.checkForObstacle(laserScan);

    ASSERT_TRUE(obstacle_detected);

}

TEST(Laser, checkNoWall){
    VelocityControlHelper vch;

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("a4_path_following");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    // The file is called smiley_og.png
    std::string file = path + "laser_no_wall.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScanPtr laserScan;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
        if (laserScan != nullptr){
            // Now we have a laserScan so we can proceed
            break;
        }
    }

    bag.close();

    ASSERT_NE(laserScan, nullptr);

    bool obstacle_detected = vch.checkForObstacle(laserScan);

    ASSERT_FALSE(obstacle_detected);

}

TEST(Laser, checkWall){
    VelocityControlHelper vch;

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("a4_path_following");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    // The file is called smiley_og.png
    std::string file = path + "laser_wall.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScanPtr laserScan;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
        if (laserScan != nullptr){
            // Now we have a laserScan so we can proceed
            break;
        }
    }

    bag.close();

    ASSERT_NE(laserScan, nullptr);

    bool obstacle_detected = vch.checkForObstacle(laserScan);

    ASSERT_TRUE(obstacle_detected);

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    
    return RUN_ALL_TESTS();
}