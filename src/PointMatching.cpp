
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
ControlError controlErrorOfTransformation(geometry_msgs::Transform transformation)
{
    return ControlError(transformation.translation.y, transformation.translation.x,
                        geo_util::quatTo2dYaw(transformation.rotation));
}

sensor_msgs::PointCloud2 applyTransform(const sensor_msgs::PointCloud2& cloud,
                                        PM::TransformationParameters transform)
{
    if(!validateTransformation(transform))
    {
        ROS_ERROR("Invalid transformation applied to reference point cloud.");
    }

    DP datapointsOfMsg = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud);

    DP transformedDataPoints = applyTransform(datapointsOfMsg, transform);

    return PointMatcher_ros::pointMatcherCloudToRosMsg<float>(transformedDataPoints,
                                                              WORLD_FRAME, ros::Time(0));
}

DP applyTransform(DP cloud, PM::TransformationParameters transform)
{
    PM::ICP icp;
    icp.setDefault();

    icp.transformations.apply(cloud, transform);

    return cloud;
}

}

