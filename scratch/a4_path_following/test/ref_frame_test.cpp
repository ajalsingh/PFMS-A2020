#include <gtest/gtest.h>
#include <climits>

#include "../src/ref_frame_conversion.h"
#include "geometry_msgs/Point.h"


TEST(RefFrame, GlobaltoLocal){
  RefFrameConversion rfc;

  geometry_msgs::Point global;
  global.x = 5;
  global.y = 4;
  geometry_msgs::Point veh;
  veh.x = 1;
  veh.y = 1;
  geometry_msgs::Point local = rfc.globalToLocal(global, veh);

  ASSERT_EQ(4, local.x);
  ASSERT_EQ(3, local.y);

  global.x = -6;
  global.y = 4;
  veh.x = -3;
  veh.y = 7;
  local = rfc.globalToLocal(global, veh);

  ASSERT_EQ(-3, local.x);
  ASSERT_EQ(-3, local.y);
}

TEST(RefFrame, LocaltoPixel){
  RefFrameConversion rfc;

  geometry_msgs::Point local;
  local.x = -3;
  local.y = -5;

  ASSERT_EQ(70, rfc.localToPixel(local).first);
  ASSERT_EQ(150, rfc.localToPixel(local).second);

  local.x = 9;
  local.y = -2;

  ASSERT_EQ(190, rfc.localToPixel(local).first);
  ASSERT_EQ(120, rfc.localToPixel(local).second);
}

TEST(RefFrame, GlobalToPixel){
  RefFrameConversion rfc;

  geometry_msgs::Point global;
  global.x = 2;
  global.y = 5;
  geometry_msgs::Point veh;
  veh.x = 3;
  veh.y = -1;

  ASSERT_EQ(90, rfc.globalToPixel(global, veh).first);
  ASSERT_EQ(40, rfc.globalToPixel(global, veh).second);
}

TEST(RefFrame, PixelToLocal){
  RefFrameConversion rfc;

  double px = 135;
  double py = 65;
  geometry_msgs::Point local = rfc.PixelToLocal(px, py);

  ASSERT_EQ(3.5, local.x);
  ASSERT_EQ(3.5, local.y);

  px = 20;
  py = 142;
  local = rfc.PixelToLocal(px, py);

  ASSERT_EQ(-8, local.x);
  ASSERT_EQ(-4.2, local.y);
}

TEST(RefFrame, LocalToGlobal){
  RefFrameConversion rfc;

  geometry_msgs::Point local;
  local.x = 5;
  local.y = 4;
  geometry_msgs::Point veh;
  veh.x = 1;
  veh.y = 1;
  geometry_msgs::Point global = rfc.LocalToGlobal(local, veh);

  ASSERT_EQ(6, global.x);
  ASSERT_EQ(5, global.y);

  local.x = -6;
  local.y = 4;
  veh.x = -3;
  veh.y = 7;
  global = rfc.LocalToGlobal(local, veh);

  ASSERT_EQ(-9, global.x);
  ASSERT_EQ(11, global.y);
}

TEST(RefFrame, PixelToGlobal){
  RefFrameConversion rfc;

  double px = 150;
  double py = 88;
  geometry_msgs::Point veh;
  veh.x = 1;
  veh.y = 1;
  geometry_msgs::Point global = rfc.pixelToGlobal(px, py, veh);

  ASSERT_EQ(6, global.x);
  ASSERT_EQ(2.2, global.y);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    
    return RUN_ALL_TESTS();
}