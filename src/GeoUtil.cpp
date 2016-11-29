#include "husky_trainer/GeoUtil.h"

#define GEO_UTIL_SEP ","
#define SCAN_RADIUS_BALLPARK 1.0

namespace geo_util
{

    double euclidian_distance_of_poses(const geometry_msgs::Pose& from, const geometry_msgs::Pose& to) {
        Eigen::Vector3d vector = vectorOfPoints(from.position, to.position);
        return vector.norm();
    }

    double angle_between_poses(const geometry_msgs::Pose& from, const geometry_msgs::Pose& to) {
        return quatTo2dYaw(
            transFromQuatToQuat(
                rosQuatToEigenQuat(from.orientation),
                rosQuatToEigenQuat(to.orientation)));
    }


Eigen::Transform<double,3,Eigen::Affine> eigenTransformOfPoses(geometry_msgs::Pose from, geometry_msgs::Pose to)
{
    Eigen::Quaternionf fromQuat = rosQuatToEigenQuat(from.orientation);
    Eigen::Quaternionf toQuat = rosQuatToEigenQuat(to.orientation);

    Eigen::Quaternionf quaternionRotation = transFromQuatToQuat(fromQuat, toQuat);
    Eigen::Transform<float,3,Eigen::Affine> rotation(quaternionRotation);

    Eigen::Translation<double,3> translation(vectorOfPoints(from.position, to.position));
    Eigen::Transform<float,3,Eigen::Affine> T = rotation * translation.cast<float>();

    return T.cast<double>();
}

tf::Transform transFromPoseToPose(geometry_msgs::Pose from, geometry_msgs::Pose to)
{
    tf::Transform transform;
    Eigen::Transform<double,3,Eigen::Affine> eigenT = eigenTransformOfPoses(from,to);
    tf::transformEigenToTF(eigenT, transform);
    return transform;
}

PM::TransformationParameters pmTransFromPoseToPose(geometry_msgs::Pose from, geometry_msgs::Pose to)
{
    Eigen::Transform<double,3,Eigen::Affine> T = eigenTransformOfPoses(from,to);

    return  T.matrix().cast<float>();
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
    Eigen::Quaternionf quatC(quat);
    quatC.x() = 0.0;
    quatC.y() = 0.0;
    quatC.normalize();

    return quatC.z() * quatC.w() >= 0 ? 2*acos(quatC.w()) : -2*acos(quatC.w());
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

Eigen::Vector3d vectorOfPoints(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs)
{
    return Eigen::Vector3d(rhs.x - lhs.x, rhs.y - lhs.y, rhs.z - lhs.z);
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

geometry_msgs::TwistStamped stampedTwistOfString(std::string in)
{
    std::stringstream ss(in);
    std::string buffer;

    std::getline(ss,buffer, ',');
    double time = strtod(buffer.c_str(), NULL);

    std::getline(ss,buffer, ',');
    double linear = strtod(buffer.c_str(), NULL);

    std::getline(ss,buffer, ',');
    double angular = strtod(buffer.c_str(), NULL);

    geometry_msgs::Vector3 linearSpeed;
    linearSpeed.x = linear;
    linearSpeed.y = 0.0;
    linearSpeed.z = 0.0;

    geometry_msgs::Vector3 angularSpeed;
    angularSpeed.x = 0.0;
    angularSpeed.y = 0.0;
    angularSpeed.z = angular;

    geometry_msgs::Twist command;
    command.linear = linearSpeed;
    command.angular = angularSpeed;

    std_msgs::Header header;
    header.stamp = ros::Time(time);

    geometry_msgs::TwistStamped msg;
    msg.twist = command;
    msg.header = header;

    return msg;
}

geometry_msgs::PoseStamped stampedPoseOfString(std::string in)
{
    std::string lineBuffer;

    std::stringstream ss(in);
    std::string timeString;
    std::getline(ss, timeString, ',');
    double time = strtod(timeString.c_str(), NULL);

    std::getline(ss, lineBuffer);

    geometry_msgs::Pose pose = stringToPose(lineBuffer);
    std_msgs::Header header;
    header.stamp = ros::Time(time);

    geometry_msgs::PoseStamped retVal;
    retVal.header = header;
    retVal.pose = pose;

    return retVal;
}

double linInterpolation(double x1, double y1, double x2, double y2, double t)
{
    double slope = (y2 - y1) / (x2 - x1);
    return slope * (t - x1) + y1;
}

geometry_msgs::Pose linInterpolation(geometry_msgs::PoseStamped lhs, geometry_msgs::PoseStamped rhs, ros::Time time)
{
    double newW = geo_util::linInterpolation(lhs.header.stamp.toSec(), lhs.pose.orientation.w,
                                             rhs.header.stamp.toSec(), rhs.pose.orientation.w,
                                             time.toSec());
    double newX = geo_util::linInterpolation(lhs.header.stamp.toSec(), lhs.pose.orientation.x,
                                             rhs.header.stamp.toSec(), rhs.pose.orientation.x,
                                             time.toSec());
    double newY = geo_util::linInterpolation(lhs.header.stamp.toSec(), lhs.pose.orientation.y,
                                             rhs.header.stamp.toSec(), rhs.pose.orientation.y,
                                             time.toSec());
    double newZ = geo_util::linInterpolation(lhs.header.stamp.toSec(), lhs.pose.orientation.z,
                                             rhs.header.stamp.toSec(), rhs.pose.orientation.z,
                                             time.toSec());

    geometry_msgs::Quaternion newQuat;
    newQuat.x = newX;
    newQuat.y = newY;
    newQuat.z = newZ;
    newQuat.w = newW;

    newX = geo_util::linInterpolation(lhs.header.stamp.toSec(), lhs.pose.position.x,
                                      rhs.header.stamp.toSec(), rhs.pose.position.x,
                                      time.toSec());
    newY = geo_util::linInterpolation(lhs.header.stamp.toSec(), lhs.pose.position.y,
                                      rhs.header.stamp.toSec(), rhs.pose.position.y,
                                      time.toSec());
    newZ = geo_util::linInterpolation(lhs.header.stamp.toSec(), lhs.pose.position.z,
                                      rhs.header.stamp.toSec(), rhs.pose.position.z,
                                      time.toSec());

    geometry_msgs::Point newPos;
    newPos.x = newX;
    newPos.y = newY;
    newPos.z = newZ;

    geometry_msgs::Pose retVal;
    retVal.orientation = newQuat;
    retVal.position = newPos;

    return retVal;
}

}
