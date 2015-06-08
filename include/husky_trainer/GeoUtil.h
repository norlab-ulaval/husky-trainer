#ifndef GEOUTIL_H
#define GEOUTIL_H

#include <string>
#include <iostream>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include "pointmatcher/PointMatcher.h"

typedef PointMatcher<float> PM;

namespace geo_util {

Eigen::Quaternionf transFromQuatToQuat(Eigen::Quaternionf from, Eigen::Quaternionf to);
double quatTo2dYaw(const geometry_msgs::Quaternion quat);
double quatTo2dYaw(const Eigen::Quaternionf quat);
PM::TransformationParameters transFromPoseToPose(geometry_msgs::Pose from, geometry_msgs::Pose to);
double customDistance(const geometry_msgs::Pose& lhs, const geometry_msgs::Pose& rhs);
std::string poseToString(geometry_msgs::Pose pose);
geometry_msgs::Pose stringToPose(std::string);
Eigen::Quaternionf rosQuatToEigenQuat(geometry_msgs::Quaternion rosQuat);
Eigen::Vector3f vectorOfPoints(geometry_msgs::Point lhs, geometry_msgs::Point rhs);

}


#endif // GEOUTIL_H
