#include <gtest/gtest.h>
#include <climits>

#include "../src/geometry.h" //This is the path of the header file in our project

TEST(geometry, distance){
    Geometry g;
    
    // Determine distance from p1(3,6) to p2(-8, 1)
    std::pair<double,double> dist = g.getDistToGoal(3,6,-8,1);

    ASSERT_EQ(dist.first, -11);
    ASSERT_EQ(dist.second, -5);

    dist = g.getDistToGoal(1,1,3,7);

    ASSERT_EQ(dist.first, 2);
    ASSERT_EQ(dist.second, 6);

    dist = g.getDistToGoal(7,1,5,7);

    ASSERT_EQ(dist.first, -2);
    ASSERT_EQ(dist.second, 6);
}

TEST(geometry, angleToGoal){
    Geometry g;
    
    // Test the angle from start angle/orientation to goal in global space
    double angle = g.getAngleToGoal(4,5,1.798);
    ASSERT_NEAR(-0.901633745, angle, 0.01);

    angle = g.getAngleToGoal(-7,3,-2.09);
    ASSERT_NEAR(-1.452, angle, 0.01);

    angle = g.getAngleToGoal(-1,-5,1.396);
    ASSERT_NEAR(3.11873, angle, 0.01);

    angle = g.getAngleToGoal(5,-6,-2.71);
    ASSERT_NEAR(1.829, angle, 0.01);
}

TEST(geometry, angleToGoalYaw){
    Geometry g;

    // Test the angle difference from current yaw to goal yaw
    double angle = g.getAngleDiffGoalYaw(M_PI_2, 3);
    ASSERT_NEAR(-1.429, angle, 0.01);

    angle = g.getAngleDiffGoalYaw(M_PI_2, -0.3491);
    ASSERT_NEAR(1.91986, angle, 0.01);

    angle = g.getAngleDiffGoalYaw(3*M_PI_4, -3*M_PI_4);
    ASSERT_NEAR(-M_PI_2, angle, 0.01);

    angle = g.getAngleDiffGoalYaw(M_PI_2, -M_PI_2);
    ASSERT_NEAR(M_PI, angle, 0.01);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    
    return RUN_ALL_TESTS();
}