
#ifndef POINTMATCHING_H
#define POINTMATCHING_H

#include <string>

#include <Eigen/Geometry>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>

#include <pointmatcher_ros/point_cloud.h>

#include "pointmatcher/PointMatcher.h"

#include "husky_trainer/GeoUtil.h"

// The errors are as following: e_x, e_y, e_theta.
typedef boost::tuple<float,float,float> ControlError;

PointMatcher<float>::DataPoints applyTransform(PointMatcher<float>::DataPoints cloud,
                                               PointMatcher<float>::TransformationParameters transform);
bool validateTransformation(PointMatcher<float>::TransformationParameters t);
ControlError controlErrorOfTransformation(geometry_msgs::Transform transformation);
sensor_msgs::PointCloud2 applyTransform(sensor_msgs::PointCloud2ConstPtr cloud,
                                        PointMatcher<float>::TransformationParameters transform);

#endif
