#include "interceptBogies.h"

InterceptBogies::InterceptBogies(const std::shared_ptr<Simulator> & sim ){
    sim_ = sim;
    f_data_.setSimulator(sim_);
    f_control_.setSimulator(sim_);

    previous_bogie_.resize(4);
    bogies_container_.vector.resize(4);
}

InterceptBogies::~InterceptBogies(){

}

void InterceptBogies::controlThread(){
    while(true){
        double lin, ang;

        //  lock to allow access to friendly_pose pose
        std::unique_lock<std::mutex> frnd_lck(friendly_pose_.mu);
        
        //  check state of aircraft and either recover or chase bogies
        AircraftState state = p_data_.checkState(friendly_pose_.pose);
        if (state == BS_UNKNOWN){
            lin = f_control_.keepInsideAirspace(friendly_pose_.pose).first;
            ang = f_control_.keepInsideAirspace(friendly_pose_.pose).second;
        }
        else{
            lin = f_control_.destroyBogie(bogies_container_.vector, friendly_pose_.pose).first;
            ang = f_control_.destroyBogie(bogies_container_.vector, friendly_pose_.pose).second;
        }
        friendly_pose_.mu.unlock();

        // control friendly
        sim_->controlFriendly(lin, ang);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void InterceptBogies::scanThread(){
    while (true){
        //  Lock thread to prevent access to raw data by process thread. Notify process thread when ready
        std::unique_lock<std::mutex> thread_lck(mu_);

        // get friendly rbs data
        std::unique_lock<std::mutex> raw_lck(raw_rbs_.mu);
        raw_rbs_.rbs = f_data_.getDataFromFriendly();
        raw_rbs_.mu.unlock();
        
        //get friendly pose data
        std::unique_lock<std::mutex> frnd_lck(friendly_pose_.mu);
        friendly_pose_.pose = f_data_.getFriendlyPose();
        friendly_pose_.mu.unlock();

        mu_.unlock();
        process_ready_ = true;
        cv_.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}

void InterceptBogies::processThread(){
    while (true){
        //lock and wait for scan thread to finish scanning data
        std::unique_lock<std::mutex> thread_lck(mu_);
        cv_.wait(thread_lck, [&]{return (process_ready_);});

        // create a vector of poses (for nearest bogie pose) and lock to access friendly data
        std::vector<Pose> poses;
        std::unique_lock<std::mutex> frnd_lck(friendly_pose_.mu);
        this->determineBogieParams(raw_rbs_.rbs, friendly_pose_.pose);
        poses.push_back(bogies_container_.vector.at(0).FuturePose);

        // Display test pose - nearest bogie
        sim_->testPose(poses);

        friendly_pose_.mu.unlock();
        process_ready_ = false;
        mu_.unlock();
    }
}

void InterceptBogies::determineBogieParams(std::vector<RangeBearingStamped> &raw_rbs, const Pose& friend_pose){
    //sort rangeBearing data by range 
    sort(raw_rbs.begin(), raw_rbs.end(), [](const RangeBearingStamped& lhs, const RangeBearingStamped& rhs) {
        return lhs.range < rhs.range;
    });

    //store sorted data as new vector
    BogieRBS sorted_rbs;
    sorted_rbs.rbs = raw_rbs;
    bogies_container_.vector.resize(4);
    for (unsigned int i=0; i<sorted_rbs.rbs.size();i++){
        //get range and bearing of bogie in global scale
        BogieCraft bogie;
        const double bogie_range = sorted_rbs.rbs.at(i).range;
        double bogie_bearing = sorted_rbs.rbs.at(i).bearing + friendly_pose_.pose.orientation;
        bool bearing_already_altered = false;
        double target_x, target_y;

        //alter bearing to max 2 pi and min 0
        if (bogie_bearing > 2*M_PI && !(bearing_already_altered)){
            bogie_bearing = bogie_bearing - 2*M_PI;
            bearing_already_altered = true;
        }
        else if (bogie_bearing < 0 && !(bearing_already_altered)){
            bogie_bearing = bogie_bearing+ 2*M_PI;
            bearing_already_altered = true;
        }

        //check where bogie lies in plane
        //first quadrant
        if (bogie_bearing <= M_PI_2 && bogie_bearing >= 0){
            target_x = bogie_range * cos(bogie_bearing);
            target_y = bogie_range * sin(bogie_bearing);
        }
        //second quadrant
        else if (bogie_bearing > M_PI_2 && bogie_bearing <= M_PI){
            target_x = -bogie_range * cos(M_PI - bogie_bearing);
            target_y = bogie_range * sin(M_PI - bogie_bearing);
        }
        //third quadrant
        else if (bogie_bearing > M_PI && bogie_bearing <= M_PI*1.5){
            target_x = -bogie_range * cos(bogie_bearing - M_PI);
            target_y = -bogie_range * sin(bogie_bearing - M_PI);
        }
        //fourth quadrant
        else if (bogie_bearing > M_PI*1.5 && bogie_bearing <= M_PI*2){
            target_x = bogie_range * cos(2*M_PI - bogie_bearing);
            target_y = -bogie_range * sin(2*M_PI - bogie_bearing);
        }

        bogie.pose.position.x = target_x + friendly_pose_.pose.position.x;
        bogie.pose.position.y = target_y + friendly_pose_.pose.position.y;
        bogie.pose.orientation = p_data_.calcOrientation(bogie.pose, previous_bogie_.at(i));
        bogie.FuturePose = p_data_.predictBogiePose(bogie.pose, previous_bogie_.at(i), 1.5);
        bogie.timeStamp = sorted_rbs.rbs.at(i).timestamp;
        bogie.rangeFromFriendly = p_data_.getFutureRangeBearingToFriend(bogie.pose, bogie.FuturePose, friendly_pose_.pose).second;
        bogie.bearingFromFriendly = sorted_rbs.rbs.at(i).bearing + p_data_.getFutureRangeBearingToFriend(bogie.pose, bogie.FuturePose, friendly_pose_.pose).first;
        bogie.state = p_data_.checkState(bogie.pose);

        bogies_container_.vector[i] = bogie;
        previous_bogie_[i] = bogie.pose;
    }
}