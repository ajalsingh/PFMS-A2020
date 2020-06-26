#include "velocity_controller.h"

VelocityController::VelocityController(ros::NodeHandle nh)
    : nh_(nh)//, refframeconversion_(nh)
{
    // Subscribing to posearray
    path_sub_ = nh_.subscribe("/robot_0/path", 1000, &VelocityController::pathCallback,this);
    
    // Subscribe to Odometry
    odom_sub_ = nh_.subscribe("robot_0/odom", 1000, &VelocityController::odomCallback,this);

    // Subscribe to Laser data
    base_scan_sub_ = nh_.subscribe("robot_0/base_scan", 1000, &VelocityController::laserCallback, this);
    
    // Publish twist velocity commands
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);

    // Publish raw path (after current goal completed)
    raw_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/raw_path",1);

    // Publish path (after current goal completed)
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/robot_0/path",1);

    // Publish completed path 
    compl_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/robot_0/completed_path",1);

    // Publish the current goal (to be used by rviz for visualisation)
    current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_0/current_goal",1);

    // param from launch file    
    ros::NodeHandle pn("~");
    pn.param<bool>("pure_pursuit", velocity_.purePursuit, false);
}

VelocityController::~VelocityController(){
    ros::NodeHandle pn("~");
    pn.deleteParam("pure_pursuit");
}

void VelocityController::pathCallback(const geometry_msgs::PoseArrayPtr &msg){
    // Check if msg is not empty
    if (!msg->poses.empty()){
        ROS_INFO_STREAM(" ");
        ROS_INFO("[VelocityController]: New goal received");
        ROS_INFO("[VelocityController]: Computing control action ...");
    

        std::unique_lock<std::mutex> lck(checkpoints_buffer_.mtx);
        checkpoints_buffer_.path.poses.clear();
        
        // transfer incoming poses to container
        for (int i = 0; i < msg->poses.size(); i++){
            checkpoints_buffer_.path.poses.push_back(msg->poses.at(i));
        }
        checkpoints_buffer_.path.header = msg->header;
        checkpoints_buffer_.received = true;

        // Set goal pose
        goal_buffer_.mtx.lock();
        goal_buffer_.goalPose.pose.position = checkpoints_buffer_.path.poses.front().position;
        goal_buffer_.goalPose.pose.orientation = checkpoints_buffer_.path.poses.front().orientation;
        goal_buffer_.hasGoal = true;
        goal_buffer_.goalPose.header = msg->header;
        current_goal_pub_.publish(goal_buffer_.goalPose);
        goal_buffer_.mtx.unlock();

        checkpoints_buffer_.mtx.unlock();
    }

    // Not path revieved
    else{
        goal_buffer_.mtx.lock();
        goal_buffer_.hasGoal = false;
        ROS_INFO("No valid goal/path supplied");

        // Remove current goal marker from rviz
        geometry_msgs::PoseStamped p;
        p.header = checkpoints_buffer_.path.header;
        p.pose.position.x = p.pose.position.y = 1000;
        current_goal_pub_.publish(p);

        goal_buffer_.mtx.unlock();
    }
}

void VelocityController::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    geometry_msgs::Pose pose=msg->pose.pose;

    checkpoints_buffer_.mtx.lock();
    goal_buffer_.vehPose= pose;
    goal_buffer_.vehPose = pose;
    checkpoints_buffer_.mtx.unlock();
}

void VelocityController::laserCallback(const sensor_msgs::LaserScanPtr& msg){
    // Check if obstacle ahead
    if (this->checkForObstacle(msg)){
        obstacle_detected_.obstacle = true;
    }
    else{
        obstacle_detected_.obstacle = false;
    }

    // Remove current target if obstacle detected and inform user
    if (obstacle_detected_.obstacle) {
        if (!obstacle_detected_.messageDisplayed){
            ROS_INFO_STREAM("[VelocityController]: Obstacle ahead, abandoning goal.");
            obstacle_detected_.messageDisplayed = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (!checkpoints_buffer_.path.poses.empty()){
                checkpoints_buffer_.path.poses.erase(checkpoints_buffer_.path.poses.begin());
                path_pub_.publish(checkpoints_buffer_.path);
            }
            raw_path_pub_.publish(checkpoints_buffer_.path);
        }
    }
    else{
        obstacle_detected_.messageDisplayed = false;
    }
}

void VelocityController::pursueGoal(){
    //determine orientation to goal
    while (ros::ok()){
        goal_buffer_.mtx.lock();

        // Check for any obstacles        
        if (!obstacle_detected_.obstacle){

            // Check if we have a goal and it hasnt been completed
            if (goal_buffer_.hasGoal && !goal_buffer_.poseCompleted){
                double veh_yaw = tf::getYaw(goal_buffer_.vehPose.orientation);
                double goal_yaw = tf::getYaw(goal_buffer_.goalPose.pose.orientation);
                double dist_x = geometry_.getDistToGoal(goal_buffer_.vehPose.position.x, goal_buffer_.vehPose.position.y,
                                                    goal_buffer_.goalPose.pose.position.x, goal_buffer_.goalPose.pose.position.y).first;
                double dist_y = geometry_.getDistToGoal(goal_buffer_.vehPose.position.x, goal_buffer_.vehPose.position.y,
                                                    goal_buffer_.goalPose.pose.position.x, goal_buffer_.goalPose.pose.position.y).second;
                double angle_to_goalpoint = geometry_.getAngleToGoal(dist_x, dist_y, veh_yaw);


                // If goal point has not yet been reached, move forwards towards the goal point
                if (!goal_buffer_.pointReached){

                    // Point and shoot config
                    if (!velocity_.purePursuit){
                        if (angle_to_goalpoint > 0.1 ){
                            velocity_.twistMsg.linear.x = 0;
                            velocity_.twistMsg.angular.z = 0.8;
                            goal_buffer_.pointReached = false;
                        }
                        else if (angle_to_goalpoint < -0.1){
                            velocity_.twistMsg.linear.x = 0;
                            velocity_.twistMsg.angular.z = -0.8;
                            goal_buffer_.pointReached = false;
                        }
                        else{
                            velocity_.twistMsg.linear.x = 0.5;
                            velocity_.twistMsg.angular.z = 0;
                            goal_buffer_.pointReached = false;
                        }
                    }

                    // Pure pursuit config
                    else{
                        if (angle_to_goalpoint > M_PI/3){
                            velocity_.twistMsg.linear.x = 0;
                            velocity_.twistMsg.angular.z = 1;
                            goal_buffer_.pointReached = false;
                        }
                        else if(angle_to_goalpoint < -M_PI/3){
                            velocity_.twistMsg.linear.x = 0;
                            velocity_.twistMsg.angular.z = -1;
                            goal_buffer_.pointReached = false;
                        }
                        else if(angle_to_goalpoint > -M_PI/60 && angle_to_goalpoint < M_PI/60){
                            velocity_.twistMsg.linear.x = 0.5;
                            velocity_.twistMsg.angular.z = 0.0;
                            goal_buffer_.pointReached = false;
                        }
                        else{
                            // Run pure pursuit and obtain gamma value
                            pp_controller_.doPurePursuit(goal_buffer_.vehPose, goal_buffer_.goalPose.pose);
                            double gamma = pp_controller_.getGamma();
                            
                            /*  Determine proportional gain
                                Linear equation developed, y = 3x/2pi, where y is gain and x is angle difference 
                                We use angle differences between 3 - 60 degrees to obtain desireable gains of 0.025-0.58*/
                            double gain = (3*fabs(angle_to_goalpoint))/(2*M_PI);
                            
                            // Set angular velocity and apply proportional gain
                            velocity_.twistMsg.angular.z = sqrt(gain*fabs(gamma));
                                                    
                            if (angle_to_goalpoint<0){
                                velocity_.twistMsg.angular.z = -(velocity_.twistMsg.angular.z);
                            }

                            // Set linear velocity and apply inverse gain
                            velocity_.twistMsg.linear.x = fabs(velocity_.twistMsg.angular.z/gamma)*(1/gain);
                            goal_buffer_.pointReached = false;
                        }
                    }

                    // Goal point has been reached with 0.1m error
                    if ((fabs(dist_y) <= 0.1 && fabs(dist_x) <= 0.1)){
                        velocity_.twistMsg.linear.x = 0;
                        velocity_.twistMsg.angular.z = 0;
                        goal_buffer_.pointReached = true;
                    }
                } 

                // Once goal point has been reached, rotate to match goal yaw
                else{
                    double angle_to_goalyaw = geometry_.getAngleDiffGoalYaw(goal_yaw, veh_yaw);

                    // Rotate to match pose orientation
                    if (angle_to_goalyaw > M_PI/18 ){
                        velocity_.twistMsg.linear.x = 0;
                        velocity_.twistMsg.angular.z = 1;
                    }
                    else if (angle_to_goalyaw < -M_PI/18){
                        velocity_.twistMsg.linear.x = 0;
                        velocity_.twistMsg.angular.z = -1;
                    }
                    else if (angle_to_goalyaw < M_PI/18 && angle_to_goalyaw > M_PI/60){
                        velocity_.twistMsg.linear.x = 0;
                        velocity_.twistMsg.angular.z = M_PI/6;
                    }
                    else if (angle_to_goalyaw < -M_PI/18 && angle_to_goalyaw > -M_PI/60){
                        velocity_.twistMsg.linear.x = 0;
                        velocity_.twistMsg.angular.z = -M_PI/6;
                    }
                    else{
                        velocity_.twistMsg.linear.x = 0;
                        velocity_.twistMsg.angular.z = 0;
                        
                        goal_buffer_.hasGoal = false;
                        goal_buffer_.poseCompleted = true;
                    }
                }
            }
        }

        // Obstacle detected
        else{
            // Obstacle detected by laser
            if (obstacle_detected_.obstacle){
                velocity_.twistMsg.linear.x = -0.5;
                velocity_.twistMsg.angular.z = 0;
            }
        }
        
        // Publish twist message to move robot
        if (goal_buffer_.hasGoal || obstacle_detected_.obstacle){
            vel_pub_.publish(velocity_.twistMsg);
        }

        // After the robot has reached goal pose (position and yaw)
        if (goal_buffer_.poseCompleted){
            if (!checkpoints_buffer_.path.poses.empty()){
                // Add goal pose to completed path array and publish
                completed_checkpoints_buffer_.path.poses.push_back(checkpoints_buffer_.path.poses.front());
                completed_checkpoints_buffer_.path.header = checkpoints_buffer_.path.header;
                compl_path_pub_.publish(completed_checkpoints_buffer_.path);
                
                // Remove completed pose from queue and publish 
                checkpoints_buffer_.path.poses.erase(checkpoints_buffer_.path.poses.begin());
                
                /*  Publish the remaining path directly to robot_0/path 
                    to allow other tracking if pathManager node is not used.
                    This message will be overwritten if pathManager node is used. */
                path_pub_.publish(checkpoints_buffer_.path);
                
                // Send msg to pathManager to determine next reachable goal
                raw_path_pub_.publish(checkpoints_buffer_.path);

                // Re-assign parameters to pursue next goal
                goal_buffer_.hasGoal = true;
                goal_buffer_.goalPose.pose = checkpoints_buffer_.path.poses.front();
            }

            goal_buffer_.poseCompleted = false;
            goal_buffer_.pointReached = false;
        }
        goal_buffer_.mtx.unlock();
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_controller");

  ros::NodeHandle nh;

  std::shared_ptr<VelocityController> vc(new VelocityController(nh));
  std::thread vel(&VelocityController::pursueGoal,vc);

  ros::spin();

  ros::shutdown();
  vel.join();

  return 0;
}