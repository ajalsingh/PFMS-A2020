#include "ranger.h"

#include <chrono>
#include <random>

/**
 * @brief Default constructor sets the offset to 0 and sets model to "unknown"
 */
Ranger::Ranger():offset_(0),model_("Unknown"){}

Ranger::~Ranger(){}

//Generates raw data for sensor
/**
 * @details   Generates random range values within max and min limits.
 *          \n Number of values generated is equal to the number of samples.
 */
std::vector<double> Ranger::generateData(){
    this->setNumSamples();
    data_.clear();

    for (unsigned int i=0;i<number_of_samples_;i++){
        double random_number = this->getRandomNumber(std::chrono::system_clock::now().time_since_epoch().count());
        if (random_number >= min_range_ && random_number <= max_range_){
            data_.push_back(random_number);
        }
        else{
            i--;
        }
    }
    return data_;
}

double Ranger::getRandomNumber(double seed){
    std::default_random_engine gen(seed);
    std::normal_distribution<double> distribution(4,5);
    return distribution(gen);
}

//Essential getters for obtaining internal private variables
unsigned int Ranger::getAngularResolution(void){
    return angular_resolution_;
}

int Ranger::getOffset(void){
    return offset_;
}

unsigned int Ranger::getFieldOfView(void){
    return field_of_view_;
}

double Ranger::getMaxRange(void){
    return max_range_;
}

double Ranger::getMinRange(void){
    return min_range_;
}

SensingMethod Ranger::getSensingMethod(void){
    return sensing_method_;
}

std::string Ranger::getModel(void){
    return model_;
}

int Ranger::getNumSamples(void){
    return number_of_samples_;
}

//Essential setters for setting internal private variables
/**
 * Sets angular resolution according to Ranger's SensingMethod type.
 * @note    for POINT type, 10 and 30 are the only acceptable values.
 *          \n for CONE type, angular velocity will be equal to field of view.
 *          \n All other values will return false.
 */
bool Ranger::setAngularResolution(unsigned int angular_resolution){
    if (sensing_method_== POINT){
        if (angular_resolution == 10 || angular_resolution == 30){
            angular_resolution_ = angular_resolution;
            this->setNumSamples();
            return true;
        }
        else{
            return false;
        }
    }
    else if (sensing_method_ = CONE){
        angular_resolution_ = this->getFieldOfView();
        this->setNumSamples();
        return true;
    }
}

/**
 * @note -180 <= offset <= 180 degrees.
 *       Values out of range will return false.
 */
bool Ranger::setOffset(int offset){
    if (offset >= -180 && offset <=180){
        offset_ = offset;
        return true;
    }
    else{
        return false;
    }
}

bool Ranger::setFieldOfView(unsigned int field_of_view){
    field_of_view_ = field_of_view;
}

void Ranger::setModel(std::string model){
    model_ = model;
}

bool Ranger::setMaxRange(double max_range){
    max_range_ = max_range;
}

bool Ranger::setMinRange(double min_range){
    min_range_ = min_range;
}

bool Ranger::setSensingMethod(SensingMethod sensing_method){
    sensing_method_ = sensing_method;
}

/**
 * @details     Number of samples for POINT type is influenced by field of view and angular resolution.
 *           \n Number of samples for CONE type is 1
 * @note        This method is automatically called after angualar resolution is set
 */
bool Ranger::setNumSamples(){
    if (sensing_method_ == POINT){
        number_of_samples_ = (field_of_view_/angular_resolution_)+1;
    }
    else{
        number_of_samples_ = 1;
    }
}
