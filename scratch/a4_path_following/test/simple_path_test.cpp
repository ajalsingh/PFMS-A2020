#include <gtest/gtest.h>
#include <climits>

#include <ros/package.h> //This tool allows to identify the path of the package on your system

#include "../src/image_processing.h" //This is the path of the header file in our project

TEST(LoadedImage, Identical){
  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("a4_path_following");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "OGMap.png";
  std::string file2 = path + "OGMap2.png";

  cv::Mat image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)
  cv::Mat image2 = cv::imread(file2,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);// Test if aquare image
  ASSERT_EQ(image.channels(),1);    // Test if aingle channel
  ASSERT_EQ(image2.rows, image2.cols);// Test if aquare image
  ASSERT_EQ(image2.channels(),1);    // Test if aingle channel

  //! Create an object of image processing as we will use the public function of that object to run tests against
  ImageProcessing imageProcessing(image,true);

  ASSERT_FALSE(imageProcessing.getSimilarity(image, image2));
  ASSERT_TRUE(imageProcessing.getSimilarity(image, image));
}

TEST(Identical, Inflated){
  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("a4_path_following");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "square.png";
  std::string file2 = path + "inflated_square.png";

  cv::Mat image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)
  cv::Mat image2 = cv::imread(file2,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);// Test if aquare image
  ASSERT_EQ(image.channels(),1);    // Test if aingle channel
  ASSERT_EQ(image2.rows, image2.cols);// Test if aquare image
  ASSERT_EQ(image2.channels(),1);    // Test if aingle channel

  //! Create an object of image processing as we will use the public function of that object to run tests against
  ImageProcessing imageProcessing(image,true);
  imageProcessing.setImage(image);
  cv::Mat image3 = imageProcessing.inflateObstacles();

  ASSERT_TRUE(imageProcessing.getSimilarity(image2, image3));
}

TEST(Identical, InflatedMap){
  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("a4_path_following");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "fake_map.png";
  std::string file2 = path + "inflated_fake_map.png";

  // Original image
  cv::Mat image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)
  
  // Expected Inflation
  cv::Mat image2 = cv::imread(file2,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);// Test if aquare image
  ASSERT_EQ(image.channels(),1);    // Test if aingle channel
  ASSERT_EQ(image2.rows, image2.cols);// Test if aquare image
  ASSERT_EQ(image2.channels(),1);    // Test if aingle channel

  //! Create an object of image processing as we will use the public function of that object to run tests against
  ImageProcessing imageProcessing(image,true);
  imageProcessing.setImage(image);
  cv::Mat image3 = imageProcessing.inflateObstacles();

  ASSERT_TRUE(imageProcessing.getSimilarity(image2, image3));
}

TEST(CanConnect, Freespace){
  //! The below code tests line connectivity between two points in clear free space

  std::string path = ros::package::getPath("a4_path_following");
  path += "/test/samples/";
  std::string file = path + "OGMap.png";

  cv::Mat image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);// Test if aquare image
  ASSERT_EQ(image.channels(),1);    // Test if single channel

  ImageProcessing imageProcessing(image,true);

  {
    cv::Point org(100,100);
    cv::Point dest(160,105);

    ASSERT_TRUE(imageProcessing.checkConnectivity(org,dest));
  }

  file = path + "OGMap2.png";

  image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);// Test if aquare image
  ASSERT_EQ(image.channels(),1);    // Test if single channel

  imageProcessing.setImage(image);

  {
    cv::Point org(68,130);
    cv::Point dest(100,135);

    ASSERT_TRUE(imageProcessing.checkConnectivity(org,dest));
  }
}

TEST(CanConnect, Unknownspace){

  std::string path = ros::package::getPath("a4_path_following");
  path += "/test/samples/";
  std::string file = path + "OGMap.png";

  cv::Mat image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);// Test if aquare image
  ASSERT_EQ(image.channels(),1);    // Test if single channel

  ImageProcessing imageProcessing(image,true);

    //! The below code tests line connectivity from free space to unknown space
  {
    cv::Point org(140,70);
    cv::Point dest(180,70);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest));
  }

  //! The below code tests line connectivity from unknown space to unknown space  {
  {  
    cv::Point org(20,170);
    cv::Point dest(90,180);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest));
  }
}

TEST(CanConnect, Occupiedspace){

  std::string path = ros::package::getPath("a4_path_following");
  path += "/test/samples/";
  std::string file = path + "OGMap2.png";

  cv::Mat image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);// Test if aquare image
  ASSERT_EQ(image.channels(),1);    // Test if single channel

  ImageProcessing imageProcessing(image,true);

    //! The below code tests line connectivity through a wall
  {
    cv::Point org(64,51);
    cv::Point dest(76,51);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest));
  }

  //! The below code tests line connectivity from either side of an occupied pixel in free space  {
  {  
    cv::Point org(56,91);
    cv::Point dest(56,94);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest));
  }

  //! The below code tests line connectivity through walls and unknown space
  {  
    cv::Point org(110,58);
    cv::Point dest(130,58);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest));
  }
}

TEST(CanConnect, Path){

  std::string path = ros::package::getPath("a4_path_following");
  path += "/test/samples/";
  std::string file = path + "OGMap3.png";

  cv::Mat image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);// Test if aquare image
  ASSERT_EQ(image.channels(),1);    // Test if single channel

  ImageProcessing imageProcessing(image,true);

    //! The below code tests line connectivity through a wall
  {
    cv::Point org(100,100);
    cv::Point dest0(130,111);
    cv::Point dest1(118,85);
    cv::Point dest2(90,105);
    cv::Point dest3(104,75);
    cv::Point dest4(150,130);

    ASSERT_TRUE(imageProcessing.checkConnectivity(org,dest0));
    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest1));
    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest2));
    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest3));
    ASSERT_FALSE(imageProcessing.checkConnectivity(org,dest4));
  }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}