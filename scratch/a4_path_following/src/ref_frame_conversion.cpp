#include "ref_frame_conversion.h"

RefFrameConversion::RefFrameConversion(): debug_(true){
    // Apply debug controls
    resolution_ = 0.1;
    mapSize_ = 200;
}

RefFrameConversion::RefFrameConversion(ros::NodeHandle nh): nh_(nh), debug_(false){
    //Below get's parameters from parameter server
    nh_.getParam("/local_map/map_resolution", resolution_);
    nh_.getParam("/local_map/map_width", mapSize_);
}

RefFrameConversion::~RefFrameConversion(){

}

geometry_msgs::Point RefFrameConversion::globalToLocal(const geometry_msgs::Point global, const geometry_msgs::Point vehicle){
    double local_x = global.x - vehicle.x;
    double local_y = global.y - vehicle.y;

    geometry_msgs::Point local;
    local.x = local_x;
    local.y = local_y;
    return local;
}

std::pair<double, double> RefFrameConversion::localToPixel(const geometry_msgs::Point local){
    double pixel_x = (local.x/resolution_) + mapSize_/2;
    double pixel_y = (-local.y/resolution_) + mapSize_/2;

    return {pixel_x, pixel_y};
}

std::pair<double, double> RefFrameConversion::globalToPixel(geometry_msgs::Point global, geometry_msgs::Point vehicle){
    return (this->localToPixel(this->globalToLocal(global, vehicle)));
}

geometry_msgs::Point RefFrameConversion::PixelToLocal(const double pixel_x, const double pixel_y){
    double local_x = (pixel_x - mapSize_/2)*resolution_;
    double local_y = -(pixel_y - mapSize_/2)*resolution_;

    geometry_msgs::Point local;
    local.x = local_x;
    local.y = local_y;
    return local;
}

geometry_msgs::Point RefFrameConversion::LocalToGlobal(const geometry_msgs::Point local, const geometry_msgs::Point vehicle){
    double global_x = local.x + vehicle.x;
    double global_y = local.y + vehicle.y;

    geometry_msgs::Point global;
    global.x = global_x;
    global.y = global_y;
    return global;
}


geometry_msgs::Point RefFrameConversion::pixelToGlobal(double pixel_x, double pixel_y, geometry_msgs::Point vehicle){
    return (this->LocalToGlobal(this->PixelToLocal(pixel_x, pixel_y),vehicle));
}

double RefFrameConversion::getmapSize(){
    return mapSize_;
}

double RefFrameConversion::getresolution(){
    return resolution_;
}