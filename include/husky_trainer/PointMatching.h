
#ifndef POINTMATCHING_H
#define POINTMATCHING_H

#include <string>

#include <Eigen/Geometry>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>

#include <pointmatcher_ros/point_cloud.h>

#include "pointmatcher/PointMatcher.h"

#include "husky_trainer/GeoUtil.h"
#include "husky_trainer/TrajectoryError.h"

namespace pointmatching_tools
{
void applyTransform(PointMatcher<float>::DataPoints& cloud,
                    PointMatcher<float>::TransformationParameters transform);
bool validateTransformation(PointMatcher<float>::TransformationParameters t);
husky_trainer::TrajectoryError controlErrorOfTransformation(geometry_msgs::Transform transformation);
sensor_msgs::PointCloud2 applyTransform(const sensor_msgs::PointCloud2 &cloud,
                                        PointMatcher<float>::TransformationParameters transform);
}
#endif
