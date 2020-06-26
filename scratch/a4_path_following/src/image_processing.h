/**
 * @file    image_processing.h
 * @ingroup Helper
 * @class   ImageProcessing
 * @brief   Handles image related operations
 * @note    Original author: Alen Alempijevic 
 *          Modified by: Ajal Singh 
 * @author  Alen Alempijevic
 * @author  Ajal Singh
 * @version 1.1
 * @date    June 2020
 */

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>

#include <vector>

//_____________________________________________________________________________ Class Definition
class ImageProcessing {

//_____________________________________________________________________________ Class Public Members
public:

  /*! @brief Default ImageProcessing constructor.
   *
   */
  ImageProcessing();

  /*! @brief ImageProcessing constructor.
   *
   *  @param image OpenCV image to be analysed
   */
  ImageProcessing(cv::Mat image, bool debug);

    /**
   * @brief Destroy the Image Processing object
   * 
   */
  ~ImageProcessing();

  /*! @brief Checks wether the origin and destination can be connected with a line, such that line only goes over free space
   *
   *  @param[in]    cv::Point oirgin - point of origin [i,j] in image coordinates
   *  @param[in]    cv::Point destination - point of destination in image coordinates
   *  @return bool  The points can be connected with a line which only goes over free space
   */
  bool checkConnectivity(const cv::Point origin, const cv::Point destination);

  /*! @brief Sets the debug and visualisation information on(true)/off(false)
    *
    *  @param[in]    bool debug - enable/disable debug
    */
  void setDebug(const bool debug);

  /*! @brief Sets the image
  *
  *  @param[in]    cv::Mat image - new image
  */
  bool setImage(const cv::Mat image);

  /**
   * @brief Get the Similarity of openCV image
   * 
   * @param A Image A
   * @param B Image B
   * @return true 
   * @return false 
   */
  bool getSimilarity(const cv::Mat A, const cv::Mat B);

  /**
   * @brief Inflates occupied cells in OGMap image
   * 
   * @return cv::Mat inflated map
   */
  cv::Mat inflateObstacles();

//_____________________________________________________________________________ Class Private Members
private:
  cv::Mat image_;                                         //!< OpenCV image structure 
  const cv::Scalar CLR_GOAL     = cv::Scalar(0,0,255);    //!< Goal colour red
  const cv::Scalar CLR_ORIGIN   = cv::Scalar(0,255,0);    //!< origin colour green
  bool debug_;                                            //!< Debug switch 

};

#endif // IMAGE_PROCESSING_H
