// Bring in my package's API, which is what I'm testing
#include "husky_trainer/PointMatching.h"
#include "husky_trainer/GeoUtil.h"
// Bring in gtest
#include <gtest/gtest.h>

#include <pointmatcher/PointMatcher.h>
#include <Eigen/Geometry>

TEST(GeoUtil, quatTo2dYaw)
{
    Eigen::Quaternionf quat1(0.669289,0.0,0.0,0.743002);
    Eigen::Quaternionf quat2(0.970984, 0.0, 0.0, 0.239143);
    double yaw1 = geo_util::quatTo2dYaw(quat1);
    double yaw2 = geo_util::quatTo2dYaw(quat2);

    std::cout << yaw1 << std::endl << yaw2 << std::endl;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
