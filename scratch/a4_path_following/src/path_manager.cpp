#include "path_manager.h"

PathManager::PathManager(ros::NodeHandle nh)
    : nh_(nh), refframeconversion_(nh)
{
    //Subscribing to rviz 2d goal pose
    pose_sub_ = nh_.subscribe("/move_base_simple/goal", 1000, &PathManager::poseCallback,this);

    // Subscribe to Odometry
    odom_sub_ = nh_.subscribe("robot_0/odom", 1000, &PathManager::odomCallback,this);

    // Subscribe to OGMap
    image_transport::ImageTransport it(nh);
    map_sub_ = it.subscribe("map_image/full", 1, &PathManager::imageCallback,this);

    //Subscribe to the raw/unordered path
    raw_path_sub_ = nh_.subscribe("/raw_path", 1000, &PathManager::rawPathCallback,this);

    // Publish path
    raw_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/raw_path",1);

    // Publish path
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/robot_0/path",1);

    duration_ = start_time_ - start_time_;
}

PathManager::~PathManager(){

}

void PathManager::poseCallback(const geometry_msgs::PoseStampedPtr &msg){
    std::unique_lock<std::mutex> lck(poses_buffer_.mtx);
    poses_buffer_.send = false;

    // Push incoming goal pose to poseArray
    poses_buffer_.path.poses.push_back(msg->pose);
    poses_buffer_.mtx.unlock();
    
    // Start timing 
    start_time_ = ros::Time::now();
    
    ROS_INFO_STREAM(" ");
    ROS_INFO_STREAM("[PathManager]: Goal Seletced. Awaiting additional goals... ");
    ROS_INFO_STREAM("[PathManager]: time-out in 5 sec");
}

void PathManager::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    geometry_msgs::Pose pose=msg->pose.pose;
    
    // Store vehicle pose
    verified_path_.mtx.lock();
    verified_path_.vehPose= pose;
    verified_path_.mtx.unlock();
}

void PathManager::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        if (enc::isColor(msg->encoding))
        cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
        else
        cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("[UpdateOGMap]: cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image;
    if((cvPtr_->image.cols !=200) || (cvPtr_->image.rows !=200)){
            ROS_WARN_THROTTLE(60, "[UpdateOGMap]: The image is not 200 x 200 what has gone wrong!");
    }
    else{
        image = cvPtr_->image;
    }

    verified_path_.mtx.lock();
    verified_path_.image = image;
    verified_path_.mtx.unlock();
}

void PathManager::rawPathCallback(const geometry_msgs::PoseArrayPtr &msg){
    std::unique_lock<std::mutex> lck(verified_path_.mtx);
    verified_path_.send = false;

    for (auto goal:msg->poses){
        verified_path_.path.poses.push_back(goal);
    }

    // Iterate through the path until a reachable goal is found (no obstruction)
    for (auto goal:msg->poses){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (this->checkGoalReachable(goal)){
            break;
        }
    } 
    
    verified_path_.send = true;

    // Prepare verified data to be published to /robot_0/path
    if (verified_path_.send){                
        verified_path_.path.header.stamp = ros::Time::now();
        verified_path_.path.header.frame_id = "/robot_0/odom";
        path_pub_.publish(verified_path_.path);
        verified_path_.path.poses.clear();
    }
    verified_path_.mtx.unlock();
}

void PathManager::publishRawPath(){
    while (ros::ok()) {
        // Check if duration since last checkpoint is > 5 secs
        duration_ = ros::Time::now() - start_time_;
        if (duration_ >= ros::Duration(5.0)){
            poses_buffer_.send = true;
            start_time_ = ros::Time::now();
        }

        // Publish path
        if (!poses_buffer_.path.poses.empty() && poses_buffer_.send){                
            poses_buffer_.path.header.stamp = ros::Time::now();
            poses_buffer_.path.header.frame_id = "/robot_0/odom";
            raw_path_pub_.publish(poses_buffer_.path);
            poses_buffer_.path.poses.clear();
        }
    }
}

bool PathManager::checkGoalReachable(const geometry_msgs::Pose goal){
    // Convert pose msg to x and y
    geometry_msgs::Point goal_point;
    goal_point.x = goal.position.x;
    goal_point.y = goal.position.y;

    // convert points to pixels
    cv::Point p1, p2;
    p1.x = refframeconversion_.getmapSize()/2;
    p1.y = refframeconversion_.getmapSize()/2;
    p2.x = refframeconversion_.globalToPixel(goal_point, verified_path_.vehPose.position).first;
    p2.y = refframeconversion_.globalToPixel(goal_point, verified_path_.vehPose.position).second;
    
    // Check if image has been set
    if (imageprocessing_.setImage(verified_path_.image)){

        // Check if goal is viable
        if (!imageprocessing_.checkConnectivity(p1, p2)){
            verified_path_.path.poses.erase(verified_path_.path.poses.begin());
            return false;
        }
        else{
            return true;
        }
    }
    else{
        ROS_INFO("image not set");
        return false;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_supplier");

  ros::NodeHandle nh;

  std::shared_ptr<PathManager> cp(new PathManager(nh));
  std::thread raw_path(&PathManager::publishRawPath,cp);

  ros::spin();

  ros::shutdown();
  raw_path.join();

  return 0;
}