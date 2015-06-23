
#include "husky_trainer/AnchorPoint.h"

#define SCAN_RADIUS_BALLPARK 10.0

const std::string AnchorPoint::POINT_CLOUD_FRAME = "/odom";

AnchorPoint::AnchorPoint() : mPointCloud()
{ }

AnchorPoint::AnchorPoint(std::string& anchorPointName, geometry_msgs::Pose position) :
    mAnchorPointName(anchorPointName), mPointCloud(), mPosition(position)
{ }

AnchorPoint::AnchorPoint(std::string& anchorPointName, 
        geometry_msgs::Pose position, sensor_msgs::PointCloud2 cloud) :
    mAnchorPointName(anchorPointName), mPointCloud(cloud), mPosition(position)
{ }


// Builds an anchor point from a string, as in the format outputted by the << operator.
AnchorPoint::AnchorPoint(std::string& anchorPointEntry)
{
    std::stringstream ss(anchorPointEntry);
    std::string buffer;
    
    std::getline(ss,buffer,',');
    std::string filename(buffer);

    std::getline(ss,buffer);

    geometry_msgs::Pose pose = geo_util::stringToPose(buffer);

    mAnchorPointName = filename;
    mPosition = pose;
    pose = getPosition();
}

AnchorPoint::~AnchorPoint()
{

}

sensor_msgs::PointCloud2 AnchorPoint::getCloud() const
{
    return mPointCloud;
}


void AnchorPoint::loadFromDisk()
{
    PointMatcher<float>::DataPoints pointCloudBuffer = PointMatcherIO<float>::loadVTK(mAnchorPointName);
    mPointCloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pointCloudBuffer, POINT_CLOUD_FRAME, ros::Time(0));
}

void AnchorPoint::saveToDisk()
{
    PointMatcher<float>::DataPoints pointCloudBuffer = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(mPointCloud);
    pointCloudBuffer.save(mAnchorPointName);
}

std::ostream& operator<<(std::ostream& out, AnchorPoint& ap)
{
    geometry_msgs::Pose pose = ap.getPosition();
    out << ap.mAnchorPointName << "," << geo_util::poseToString(pose);
    return out;
}


std::string AnchorPoint::name() const
{
    return mAnchorPointName;
}

geometry_msgs::Pose AnchorPoint::getPosition() const
{
    return mPosition;
}



