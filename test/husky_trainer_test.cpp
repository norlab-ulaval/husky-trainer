// Bring in my package's API, which is what I'm testing
#include "husky_trainer/PointMatching.h"
// Bring in gtest
#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pointmatcher/PointMatcher.h>

// Declare a test
TEST(AnchorPoint, loadPCD)
{
    PointMatcher<float>::DataPoints dp = PointMatcher<float>::DataPoints::load("sample.pcd");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
