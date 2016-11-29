#ifndef GEOUTIL_H
#define GEOUTIL_H

#include <string>
#include <iostream>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include "pointmatcher/PointMatcher.h"

typedef PointMatcher<float> PM;

namespace geo_util {

double euclidian_distance_of_poses(const geometry_msgs::Pose& from, const geometry_msgs::Pose& to);
double angle_between_poses(const geometry_msgs::Pose& from, const geometry_msgs::Pose& to);
Eigen::Quaternionf transFromQuatToQuat(Eigen::Quaternionf from, Eigen::Quaternionf to);
double quatTo2dYaw(const geometry_msgs::Quaternion quat);
double quatTo2dYaw(const Eigen::Quaternionf quat);
Eigen::Transform<double,3,Eigen::Affine> eigenTransformOfPoses(geometry_msgs::Pose from, geometry_msgs::Pose to);
PM::TransformationParameters pmTransFromPoseToPose(geometry_msgs::Pose from, geometry_msgs::Pose to);
tf::Transform transFromPoseToPose(geometry_msgs::Pose from, geometry_msgs::Pose to);
double customDistance(const geometry_msgs::Pose& lhs, const geometry_msgs::Pose& rhs);
std::string poseToString(geometry_msgs::Pose pose);
geometry_msgs::Pose stringToPose(std::string);
Eigen::Quaternionf rosQuatToEigenQuat(geometry_msgs::Quaternion rosQuat);
Eigen::Vector3d vectorOfPoints(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs);
geometry_msgs::TwistStamped stampedTwistOfString(std::string in);
geometry_msgs::PoseStamped stampedPoseOfString(std::string in);
double linInterpolation(double x1, double y1, double x2, double y2, double t);
geometry_msgs::Pose linInterpolation(geometry_msgs::PoseStamped lhs, geometry_msgs::PoseStamped rhs, ros::Time time);
}


#endif // GEOUTIL_H
