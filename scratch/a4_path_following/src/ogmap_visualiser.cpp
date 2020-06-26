#include "ogmap_visualiser.h"

OGMapVisual::OGMapVisual(ros::NodeHandle nh)
    : nh_(nh), refframeconversion_(nh)
{
    // Subscribeto incoming posearray
    path_sub_ = nh_.subscribe("/robot_0/path", 1000, &OGMapVisual::pathCallback,this);
    
    // Subscribe to completed path posearray
    compl_path_sub_ = nh_.subscribe("/robot_0/completed_path", 1000, &OGMapVisual::completedpathCallback,this);

    // Subscribe to OGMap
    image_transport::ImageTransport it(nh);
    map_sub_ = it.subscribe("map_image/full", 1, &OGMapVisual::imageCallback,this);
    
    // Subscribe to Odometry
    odom_sub_ = nh_.subscribe("robot_0/odom", 1000, &OGMapVisual::odomCallback,this);
    
    // Publish array of checkpoint Poses
    map_image_pub_ = it.advertise("/map_image/path_following",1);
}

OGMapVisual::~OGMapVisual(){
    
}

void OGMapVisual::pathCallback(const geometry_msgs::PoseArrayPtr &msg){

    std::unique_lock<std::mutex> lck(checkpoints_buffer_.mtx);
    checkpoints_buffer_.path.poses.clear();

    // transfer incoming poses to container
    for (int i = 0; i < msg->poses.size(); i++){
        checkpoints_buffer_.path.poses.push_back(msg->poses.at(i));
    }
    checkpoints_buffer_.path.header = msg->header;
    checkpoints_buffer_.received = true;

    checkpoints_buffer_.mtx.unlock();
}

void OGMapVisual::completedpathCallback(const geometry_msgs::PoseArrayPtr &msg){
    std::unique_lock<std::mutex> lck(completed_checkpoints_buffer_.mtx);
    completed_checkpoints_buffer_.path.poses.clear();

    // transfer incoming poses to container
    for (int i = 0; i < msg->poses.size(); i++){
        completed_checkpoints_buffer_.path.poses.push_back(msg->poses.at(i));
    }
    completed_checkpoints_buffer_.path.header = msg->header;
    completed_checkpoints_buffer_.received = true;

    completed_checkpoints_buffer_.mtx.unlock();
}


void OGMapVisual::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    // Set vehicle pose in container
    geometry_msgs::Pose pose=msg->pose.pose;

    checkpoints_buffer_.mtx.lock();
    checkpoints_buffer_.vehPose= pose;
    checkpoints_buffer_.mtx.unlock();
}

void OGMapVisual::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        if (enc::isColor(msg->encoding))
        cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
        else
        cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("[OGMapVisual]: cv_bridge exception: %s", e.what());
        return;
    }

    std::pair<cv::Mat, ros::Time> image_time = {cvPtr_->image, cvPtr_->header.stamp};

    raw_image_buffer_.mtx.lock();
    raw_image_buffer_.imageStampedDeq.push_back(image_time);
    if((cvPtr_->image.cols !=200) || (cvPtr_->image.rows !=200)){
            ROS_WARN_THROTTLE(60, "[OGMapVisual]: The image is not 200 x 200 what has gone wrong!");
    }

    if(raw_image_buffer_.imageStampedDeq.size()>2){
        raw_image_buffer_.imageStampedDeq.pop_front();
    }

    raw_image_buffer_.mtx.unlock();
    
    // update the image as robot moves and path msg is received 
    this->update();
}

void OGMapVisual::update(){
    ros::Time timeImage = ros::Time::now();
    cv::Mat image;

    bool imageOK=false;

    //! Lock image buffer, take one message from deque and unlock it
    raw_image_buffer_.mtx.lock();
    if(raw_image_buffer_.imageStampedDeq.size()>0){
        image = raw_image_buffer_.imageStampedDeq.front().first;
        timeImage = raw_image_buffer_.imageStampedDeq.front().second;
        raw_image_buffer_.imageStampedDeq.pop_front();
        imageOK=true;
    }
    raw_image_buffer_.mtx.unlock();

    // Update GUI Window on new OgMap received
    if(imageOK){
        // Create rgb image
        cv::Mat rgbImage;
        cv::cvtColor(image,rgbImage,CV_GRAY2RGB);

        checkpoints_buffer_.mtx.lock();

        // Draw a square representing the robot for ease of visualisation
        geometry_msgs::Point r1, r2;
        r1.x = checkpoints_buffer_.vehPose.position.x - 0.2;
        r1.y = checkpoints_buffer_.vehPose.position.y + 0.2;
        r2.x = checkpoints_buffer_.vehPose.position.x + 0.2;
        r2.y = checkpoints_buffer_.vehPose.position.y - 0.2;

        cv::Point p1, p2;
        p1.x = refframeconversion_.globalToPixel(r1, checkpoints_buffer_.vehPose.position).first;
        p1.y = refframeconversion_.globalToPixel(r1, checkpoints_buffer_.vehPose.position).second;
        p2.x = refframeconversion_.globalToPixel(r2, checkpoints_buffer_.vehPose.position).first;
        p2.y = refframeconversion_.globalToPixel(r2, checkpoints_buffer_.vehPose.position).second;
        cv::rectangle(rgbImage, p1, p2, CV_RGB(0,0,0), -1);
        
        // Only draw points if targets exist
        if (!checkpoints_buffer_.path.poses.empty()){
            
            // Transform global to pixels and draw checkpoints
            for (int i = 0; i < checkpoints_buffer_.path.poses.size(); i++){
                geometry_msgs::Pose pose = checkpoints_buffer_.path.poses.at(i);

                std::pair<double, double> pixel = refframeconversion_.globalToPixel(pose.position, checkpoints_buffer_.vehPose.position);

                if (i == 0){
                    // Draw green circle to indicate current goal
                    cv::circle(rgbImage, cv::Point(pixel.first, pixel.second), 3, CV_RGB(0,255,0), -1);
                }
                else{
                    // Draw red circle to indicate remaining goals
                    cv::circle(rgbImage, cv::Point(pixel.first, pixel.second), 3, CV_RGB(255,0,0), -1);
                }
            }
            
        }
        checkpoints_buffer_.mtx.unlock();

        completed_checkpoints_buffer_.mtx.lock();
        if (!completed_checkpoints_buffer_.path.poses.empty()){
            for (int i = 0; i < completed_checkpoints_buffer_.path.poses.size(); i++){
                geometry_msgs::Pose pose = completed_checkpoints_buffer_.path.poses.at(i);

                std::pair<double, double> pixel = refframeconversion_.globalToPixel(pose.position, checkpoints_buffer_.vehPose.position);

                // Draw blue circle to indicate completed goals
                cv::circle(rgbImage, cv::Point(pixel.first, pixel.second), 3, CV_RGB(0,0,255), -1);
            }
        }
        completed_checkpoints_buffer_.mtx.unlock();

        cv::putText(rgbImage, "Received Path.", cv::Point(30,30),
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(200,200,250), 1, CV_AA);

        // Publish image
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->image = rgbImage;
        cv_ptr->encoding = "bgr8";
        map_image_pub_.publish(cv_ptr->toImageMsg());

        cv::waitKey(5);
    }
    // This delay slows the loop down for the sake of readability
    std::this_thread::sleep_for (std::chrono::milliseconds(50));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "update_ogmap");

  ros::NodeHandle nh;

  std::shared_ptr<OGMapVisual> ogm(new OGMapVisual(nh));

  ros::spin();

  ros::shutdown();

  return 0;
}