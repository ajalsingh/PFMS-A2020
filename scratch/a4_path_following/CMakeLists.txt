cmake_minimum_required(VERSION 2.8.3)
project(a4_path_following)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# Tghis pacakge depends on a5_setup, beacuse it uses a service declared there
find_package(catkin REQUIRED COMPONENTS
  a4_setup
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  rosbag 
  roslib
)

add_definitions( -DANALYSIS=ON )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system signals)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

find_package(OpenCV)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES drawing_tools
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}-image_processing src/image_processing.cpp)
add_library(${PROJECT_NAME}-ref_frame_conversion src/ref_frame_conversion.cpp)
add_library(${PROJECT_NAME}-geometry src/geometry.cpp)
add_library(${PROJECT_NAME}-laser src/controller_helper.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}-path_manager src/path_manager.cpp )
add_executable(${PROJECT_NAME}-ogmap_visualiser src/ogmap_visualiser.cpp )
add_executable(${PROJECT_NAME}-velocity_control src/velocity_controller.cpp src/geometry.cpp src/pure_pursuit.cpp src/controller_helper.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
## This enforces generating messages before building these executables
add_dependencies(${PROJECT_NAME}-path_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}-ogmap_visualiser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}-velocity_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-path_manager ${catkin_LIBRARIES} ${PROJECT_NAME}-ref_frame_conversion ${PROJECT_NAME}-image_processing ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}-ogmap_visualiser ${catkin_LIBRARIES} ${PROJECT_NAME}-ref_frame_conversion ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}-velocity_control ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test_path test/simple_path_test.cpp )
if(TARGET ${PROJECT_NAME}-test_path)
   target_link_libraries(${PROJECT_NAME}-test_path ${catkin_LIBRARIES} ${PROJECT_NAME}-image_processing ${OpenCV_LIBRARIES})
message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
endif()

catkin_add_gtest(${PROJECT_NAME}-test_ref test/ref_frame_test.cpp )
if(TARGET ${PROJECT_NAME}-test_ref)
   target_link_libraries(${PROJECT_NAME}-test_ref ${catkin_LIBRARIES} ${PROJECT_NAME}-ref_frame_conversion ${OpenCV_LIBRARIES})
message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
endif()

catkin_add_gtest(${PROJECT_NAME}-test_geometry test/geometry_test.cpp )
if(TARGET ${PROJECT_NAME}-test_geometry)
   target_link_libraries(${PROJECT_NAME}-test_geometry ${catkin_LIBRARIES} ${PROJECT_NAME}-geometry ${OpenCV_LIBRARIES})
message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
endif()

catkin_add_gtest(${PROJECT_NAME}-test_laser test/laser_test.cpp )
if(TARGET ${PROJECT_NAME}-test_laser)
   target_link_libraries(${PROJECT_NAME}-test_laser ${catkin_LIBRARIES} ${PROJECT_NAME}-laser ${OpenCV_LIBRARIES})
message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
endif()

