
#include "husky_trainer/PointMatching.h"

#define WORLD_FRAME "/odom"

namespace pointmatching_tools
{


typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

bool validateTransformation(PM::TransformationParameters t)
{
    PM::Transformation* rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    return rigidTrans->checkParameters(t);
}

// Go from a transformation between two point clouds to an error we can apply to the controller.
// The axis convention in libpointmatcher is different than the one throughout
// the husky, which is why x is mapped to y and vice versa.
husky_trainer::TrajectoryError controlErrorOfTransformation(geometry_msgs::Transform transformation)
{
    husky_trainer::TrajectoryError error;
    error.x = transformation.translation.y;
    error.y = transformation.translation.x;
    error.theta = geo_util::quatTo2dYaw(transformation.rotation);

    return error;
}

sensor_msgs::PointCloud2 applyTransform(const sensor_msgs::PointCloud2& cloud,
                                        PM::TransformationParameters transform)
{
    if(!validateTransformation(transform))
    {
        ROS_ERROR("Invalid transformation applied to reference point cloud.");
    }

    DP datapointsOfMsg = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud);

    applyTransform(datapointsOfMsg, transform);

    return PointMatcher_ros::pointMatcherCloudToRosMsg<float>(datapointsOfMsg,
                                                              WORLD_FRAME, ros::Time(0));
}

void applyTransform(DP& cloud, PM::TransformationParameters transform)
{
    PM::ICP icp;
    icp.setDefault();
    icp.transformations.apply(cloud, transform);
}

}

