#include "image_processing.h"
#include <image_transport/image_transport.h>
#include <iostream>

ImageProcessing::ImageProcessing():
  debug_(false){
}

ImageProcessing::ImageProcessing(cv::Mat image, bool debug):
  image_(image),debug_(debug){
}

ImageProcessing::~ImageProcessing(){
  
}

void ImageProcessing::setDebug(const bool debug){
  debug_=debug;
}

bool ImageProcessing::setImage(const cv::Mat image){
  image.copyTo(image_);//This will copy the image into our member variable (image_) courtesy of OpenCV
  
  // Check if image is empty
  if(image_.empty()){
    std::cout << "No image supplied yet" << std::endl;
    return false;
  }
  else {
    return true;
  }
}

bool ImageProcessing::checkConnectivity(const cv::Point origin, const cv::Point destination)
{
  // Inflate map with respect to robot radius
  cv::Mat inflated_image = this->inflateObstacles();

  // Check image size
  if (inflated_image.cols == 0){
    std::cout<<"error reading file"<<std::endl;
    return false;
  }
  else{
    // Used in unit testing/debugging
    if(debug_){
      cv::Mat color_image;
      cv::cvtColor(inflated_image, color_image, cv::COLOR_GRAY2BGR);
      cv::circle(color_image,origin,1,CLR_ORIGIN,2);
      cv::circle(color_image,destination,1,CLR_GOAL,2);
      cv::putText(color_image, "green - origin ,red - destination", cv::Point(0,10), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(255,0,0));
      cv::namedWindow("debug",cv::WINDOW_NORMAL);
      cv::imshow("debug",color_image);
      cv::waitKey(5000);
      cv::destroyWindow("debug");
    }

    //! Create a Line Iterator  between origib abd destination
    cv::LineIterator lineIterator(inflated_image, origin, destination);

    // Search the line of white pixels
    for (int i = 0; i < lineIterator.count; i++, lineIterator++) {
        uchar *pixel = (uchar*)*lineIterator;
        if (*pixel < 255) {
            return false;
        }
    }
    return true;
  }
}

bool ImageProcessing::getSimilarity(const cv::Mat A, const cv::Mat B)
{
  // Check images are the same size
  if((A.cols != B.cols) || (A.rows != B.rows)){
    return false;
  }

  cv::Mat C;
  cv::absdiff(A, B, C);

  if (debug_){
    cv::namedWindow("debug1",cv::WINDOW_NORMAL);
    cv::imshow("debug1",A);
    
    cv::namedWindow("debug2",cv::WINDOW_NORMAL);
    cv::imshow("debug2",B);
    
    cv::namedWindow("diff",cv::WINDOW_NORMAL);
    cv::imshow("diff",C);
    
    cv::waitKey(5000);
    cv::destroyWindow("debug1");
    cv::destroyWindow("debug2");
    cv::destroyWindow("diff");
  }

  // Count the non zero (white) pixels
  if (cv::countNonZero(C) > 0) {
      return false; // Not identical
  }
  else {
      return true;  // Identical
  }
}

cv::Mat ImageProcessing::inflateObstacles(){
  cv::Mat inflated_image;
  image_.copyTo(inflated_image);
  
  // Convert images to single channel if not already
  if (inflated_image.channels() != 1){
    cv::cvtColor(inflated_image, inflated_image, cv::COLOR_BGR2GRAY);
  }
  std::vector<cv::Point> inflate;
  // Iterate through each pixel
  for (int i=0;i<image_.rows;i++){
    for (int j=0;j<image_.cols;j++){
      cv::Point pix = {i,j};
      int pix_value = image_.at<uchar>(pix);

      // If pixel is black, expand
      if (pix_value == 0){
        inflate.push_back(pix);
      }
      else{
        inflated_image.at<uchar>(pix) = pix_value;
      }
    }
  }
  
  for (int i=0;i<inflate.size();i++){
    cv::rectangle(inflated_image, {inflate.at(i).x-2, inflate.at(i).y-2}, {inflate.at(i).x+2, inflate.at(i).y+2}, CV_RGB(0,255,0));
  }
  return inflated_image;
}