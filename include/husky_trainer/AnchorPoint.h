
#ifndef ANCHOR_POINT_H
#define ANCHOR_POINT_H

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/IO.h>

#include "husky_trainer/GeoUtil.h"


class AnchorPoint {
    private:
        std::string mAnchorPointName;
        PointMatcher<float>::DataPoints mPointCloud;
        geometry_msgs::Pose mPosition;

    public:
        AnchorPoint(std::string& anchorPointName, geometry_msgs::Pose position);
        AnchorPoint(std::string& anchorPointName, geometry_msgs::Pose position, PointMatcher<float>::DataPoints cloud);
        AnchorPoint(std::string& anchorPointEntry);
        AnchorPoint();
        ~AnchorPoint();

        PointMatcher<float>::DataPoints getCloud() const;
        void loadFromDisk();
        void saveToDisk();

        
        friend std::ostream& operator<<(std::ostream& out, AnchorPoint& ap);

        std::string name() const;
        geometry_msgs::Pose getPosition() const;
};

#endif
