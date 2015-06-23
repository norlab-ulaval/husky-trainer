
#ifndef ANCHOR_POINT_H
#define ANCHOR_POINT_H

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/IO.h>
#include <pointmatcher_ros/point_cloud.h>

#include "husky_trainer/GeoUtil.h"


class AnchorPoint {
private:
    const static std::string POINT_CLOUD_FRAME;

    std::string mAnchorPointName;
    sensor_msgs::PointCloud2 mPointCloud;
    geometry_msgs::Pose mPosition;

public:
    AnchorPoint(std::string& anchorPointName, geometry_msgs::Pose position);
    AnchorPoint(std::string& anchorPointName, geometry_msgs::Pose position, sensor_msgs::PointCloud2 cloud);
    AnchorPoint(std::string& anchorPointEntry);
    AnchorPoint();
    ~AnchorPoint();

    sensor_msgs::PointCloud2 getCloud() const;
    void loadFromDisk();
    void saveToDisk();


    friend std::ostream& operator<<(std::ostream& out, AnchorPoint& ap);

    std::string name() const;
    geometry_msgs::Pose getPosition() const;
};

#endif
