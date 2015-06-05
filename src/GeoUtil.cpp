#include "husky_trainer/GeoUtil.h"

#define GEO_UTIL_SEP ","
#define SCAN_RADIUS_BALLPARK 1.0

namespace geo_util
{

PM::TransformationParameters transFromPoseToPose(geometry_msgs::Pose from, geometry_msgs::Pose to)
{
    Eigen::Quaternionf fromQuat = rosQuatToEigenQuat(from.orientation);
    Eigen::Quaternionf toQuat = rosQuatToEigenQuat(to.orientation);
    Eigen::Transform<float,3,Eigen::Affine> rotation(transFromQuatToQuat(fromQuat, toQuat));

    Eigen::Translation<float,3> translation(vectorOfPoints(from.position, to.position));
    Eigen::Transform<float,3,Eigen::Affine> T = rotation * translation;

    return T.matrix();
}

Eigen::Quaternionf transFromQuatToQuat(Eigen::Quaternionf from, Eigen::Quaternionf to)
{
    from.normalize();
    to.normalize();

    Eigen::Quaternionf transformation = from * to.conjugate();

    return transformation;
}

double quatTo2dYaw(const Eigen::Quaternionf quat)
{
    return atan2(quat.toRotationMatrix()(1, 2), quat.toRotationMatrix()(2, 2));
}

// This is NOT a regular euclidian distance. This function also accounts for the
// orientation w. Furthermore w has more weight on the distance as a small
// rotation can cause big differences between two anchor points.
double customDistance(const geometry_msgs::Pose& lhs, const geometry_msgs::Pose& rhs)
{
    double lhsYaw = quatTo2dYaw(lhs.orientation);
    double rhsYaw = quatTo2dYaw(rhs.orientation);

    double distanceSquared =
        (lhs.position.x - rhs.position.x) * (lhs.position.x - rhs.position.x) +
        (lhs.position.y - rhs.position.y) * (lhs.position.y - rhs.position.y) +
        (lhs.position.z - rhs.position.z) * (lhs.position.z - rhs.position.z) +
        (lhsYaw - rhsYaw) * SCAN_RADIUS_BALLPARK *
        (lhsYaw - rhsYaw) * SCAN_RADIUS_BALLPARK;

    return sqrt(distanceSquared);
}

double quatTo2dYaw(const geometry_msgs::Quaternion quat)
{
    return geo_util::quatTo2dYaw(rosQuatToEigenQuat(quat));
}

float quatTo2dYaw(const tf::Quaternion quat)
{
    Eigen::Quaternionf eigenQuat(quat.getW(), quat.getX(), quat.getY(), quat.getZ());
    return quatTo2dYaw(eigenQuat);
}

std::string poseToString(geometry_msgs::Pose pose)
{
    std::ostringstream oss;
    oss << pose.position.x << GEO_UTIL_SEP << pose.position.y << GEO_UTIL_SEP <<
        pose.position.z << GEO_UTIL_SEP << pose.orientation.x << GEO_UTIL_SEP <<
        pose.orientation.y << GEO_UTIL_SEP << pose.orientation.z << GEO_UTIL_SEP <<
        pose.orientation.w << std::endl;
    return oss.str();
}

Eigen::Quaternionf rosQuatToEigenQuat(geometry_msgs::Quaternion rosQuat)
{
    return Eigen::Quaternionf(rosQuat.w, rosQuat.x, rosQuat.y, rosQuat.z);
}

Eigen::Vector3f vectorOfPoints(geometry_msgs::Point lhs, geometry_msgs::Point rhs)
{
    return Eigen::Vector3f(rhs.x - lhs.x, rhs.y - lhs.y, rhs.z - lhs.z);
}

geometry_msgs::Pose stringToPose(std::string input)
{
    std::stringstream ss(input);
    std::string buffer;
    std::vector<double> lineValues;

    for(int i = 0; i < 7; i++)
    {
        std::getline(ss, buffer, ',');
        lineValues.push_back(strtod(buffer.c_str(), NULL));
    }

    geometry_msgs::Point position;
    position.x = lineValues[0];
    position.y = lineValues[1];
    position.z = lineValues[2];

    geometry_msgs::Quaternion orientation;
    orientation.x = lineValues[3];
    orientation.y = lineValues[4];
    orientation.z = lineValues[5];
    orientation.w = lineValues[6];

    geometry_msgs::Pose pose;
    pose.orientation = orientation;
    pose.position = position;

    return pose;
}

}
