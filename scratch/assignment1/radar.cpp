#include "radar.h"

#include <vector>
#include <iostream>
#include <time.h>
#include <thread>
#include <chrono>

Radar::Radar(): model_("RAD-XL"), output_("Targets (Range and Bearing Data)"), field_of_view_degrees_(20), min_distance_metres_(0.5), max_number_of_targets_(20), scanning_time_(100), max_distance_(80){}

Radar::~Radar(){}

Radar::Radar(int scan_time_ms, int max_distance_metres){
    model_ = "RAD-XL";
    output_ = "Targets (Range and Bearing Data)";
    field_of_view_degrees_ = 20;
    min_distance_metres_ = 0.5;
    max_number_of_targets_ = 20;
    if (scan_time_ms == 100 && max_distance_metres == 80){      //  Ensures coupled params are linked. Incompatible configurations will not be applied
        scanning_time_ = scan_time_ms;
        max_distance_ = max_distance_metres;
    }
    if (scan_time_ms == 200 && max_distance_metres == 20){
        scanning_time_ = scan_time_ms;
        max_distance_ = max_distance_metres;
    }
}

std::string Radar::getModel(){
    return model_;
}

std::string Radar::getOutput(){
    return output_;
}

int Radar::getFieldOfViewDeg(){
    return field_of_view_degrees_;
}

double Radar::getMinDist(){
    return min_distance_metres_;
}

int Radar::getMaxTargets(){
    return max_number_of_targets_;
}

int Radar::setTargets(){
    srand((unsigned) time(0));
    number_of_targets_ = (rand() % max_number_of_targets_) + 1.0;   //  Generates random number of targets
}

int Radar::getTargets(){
    return number_of_targets_;
}

int Radar::getScanTime(){
    return scanning_time_;
}

bool Radar::setScanTime(int scan_time_ms){
    if (scan_time_ms == 100 || scan_time_ms == 200){                //  Only supported scanning times can be set
        scanning_time_ = scan_time_ms;
        this->configTimerStart();
        return true;
    }
    else{
        return false;
    }
}

double Radar::getMaxDist(){
    return max_distance_;
}

bool Radar::setMaxDist(double max_distance_metres){
    if (max_distance_metres == 80 || max_distance_metres == 20){    //  Only supported max distances can be set
        max_distance_ = max_distance_metres;
        return true;
    }
    else{
        return false;
    }
}


std::vector<double> Radar::getData(){
    return vec_range_bearing_;
}


// This method initialises the vector used to store target data (range and bearing)
// random target number is generated, for each target a range and bearing is then pushed into the vector
// After this method is called, the poll method is called to update the range values
void Radar::setData(){
    vec_range_bearing_.clear();                                     //clear vector

    this->setTargets();                                             //generate a random number of targets
    this->resetSampleCount();                                       //set sample count to 1
    vec_range_bearing_.push_back(this->getTargets());               // the first two elements in the vector hold the number of targets and sample count repectively
    vec_range_bearing_.push_back(this->getSampleCount());
    for (int i = 0; i < number_of_targets_; i++){                   //for each target, push random range and bearing value
        distance_ = (rand() % max_distance_) + 0.5;
        vec_range_bearing_.push_back(distance_);

        bearing_ = (rand() % ((2*field_of_view_degrees_)+1)) + (-field_of_view_degrees_);
        vec_range_bearing_.push_back(bearing_);
    }
    this->queryTimerStart();
}


// This method updates the values generated in setData
double Radar::poll(){
    for (int i = 2; i < vec_range_bearing_.size(); i+=2){
        vec_range_bearing_[i] = vec_range_bearing_[i] * 0.95;
        if (vec_range_bearing_[i] < 0.5){
            vec_range_bearing_[i] = 0.50001;                        //minimum value of max distance
        }
    }
    this->configTimerStop();
    this->queryTimerStop();
    sample_count_++;
    vec_range_bearing_[1] = sample_count_;
    std::this_thread::sleep_for (std::chrono::milliseconds(scanning_time_));
}


int Radar::getSampleCount(){
    return sample_count_;
}

void Radar::resetSampleCount(){
    sample_count_ = 1;
}

void Radar::configTimerStart(){
    start_config_time_point_ = std::chrono::high_resolution_clock::now();
    time_since_config_change_ = std::chrono::duration<double>::zero();
}

void Radar::configTimerStop(){
    auto stop_timer = std::chrono::high_resolution_clock::now();
    time_since_config_change_ = stop_timer - start_config_time_point_;
}

double Radar::getConfigTime(){
    return std::chrono::duration_cast<std::chrono::seconds>(time_since_config_change_).count();
}

void Radar::queryTimerStart(){
    start_query_time_point_ = std::chrono::high_resolution_clock::now();
    time_since_first_query_ = std::chrono::duration<double>::zero();
}

void Radar::queryTimerStop(){
    auto stop_timer = std::chrono::high_resolution_clock::now();
    time_since_first_query_ = stop_timer - start_query_time_point_;
}

double Radar::getQueryTime(){
    return std::chrono::duration_cast<std::chrono::seconds>(time_since_first_query_).count();
}
