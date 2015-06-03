
#include <vector>
#include <fstream>
#include <math.h>
#include <cmath>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/callback_queue.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <clearpath_base/Encoders.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "pointcloud_tools/CloudRecorder.h"

#include "husky_trainer/AnchorPoint.h"
#include "husky_trainer/PointMatching.h"


#define JOYSTICK_TOPIC "joy"
#define POINT_CLOUD_TOPIC "/cloud"
#define POSE_ESTIMATE_TOPIC "/robot_pose_ekf/odom_combined"
#define WHEEL_TRAVEL_TOPIC "/husky/data/encoders"
#define VEL_TOPIC "/husky/cmd_vel"

#define ROBOT_FRAME "/base_footprint"
#define LIDAR_FRAME "/velodyne"

#define Y_BUTTON_INDEX 3

#define ANCHOR_POINT_DISTANCE 0.01  // The approx distance we want between every anchor point.
#define LOOP_RATE 100
#define L_SEP ","

typedef PointMatcher<float> PM;

// Node global variables.
std::ofstream* pPositionRecord;
std::ofstream* pSpeedRecord;
int nextCloudIndex;
float lastTravelRecorded;
float travelOfLastAnchor;

// Path recording information.
ros::Time teachingStartTime;
std::vector<AnchorPoint> anchorPointList;
boost::mutex anchorPointListMutex;
geometry_msgs::Pose lastOdomPosition;
PM::TransformationParameters tLidarToBaseLink;
ros::ServiceClient* pCrClient;

void saveAnchorPointList(std::vector<AnchorPoint>& list)
{
    std::ofstream anchorPointListFile;
    anchorPointListFile.open("anchorPoints.apd");
   
    for(int i=0; i < anchorPointList.size(); i++)
    {
        anchorPointListFile << list[i];
    }

    anchorPointListFile.close();
}

void recordCloud(const sensor_msgs::PointCloud2& cloud)
{
    pointcloud_tools::CloudRecorder service;
    service.request.cloud = cloud;
    if(!pCrClient->call(service))
    {
        ROS_WARN("There was something wrong with the cloud recording service.");
    }

    while(!anchorPointListMutex.try_lock())
    {}

    AnchorPoint newAnchorPoint(service.response.filename, lastOdomPosition);
    anchorPointList.push_back(newAnchorPoint);

    anchorPointListMutex.unlock();
    ROS_INFO("Saved a cloud.");
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    ROS_INFO("Travel: %f", fabs(lastTravelRecorded - travelOfLastAnchor));

    if(teachingStartTime != ros::Time(0))
    {
        // Check if we traveled enough to get a new cloud, or if the travel value
        // has overflowed since the last cloud was recorded.
        if(fabs(lastTravelRecorded - travelOfLastAnchor) > ANCHOR_POINT_DISTANCE)
        {
            travelOfLastAnchor = lastTravelRecorded;

            ROS_INFO("Saving a new anchor point");

            PM::DataPoints dataPoints;
            dataPoints = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*msg);

            PM::DataPoints transformedCloud = applyTransform(dataPoints, tLidarToBaseLink);

            sensor_msgs::PointCloud2 transformedMsg = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(transformedCloud, "base_link", ros::Time(0));
            boost::thread cloudRecordingThread(recordCloud, transformedMsg);
        }
        else
        {
            ROS_INFO("Got a cloud too close to the last anchor point. Ignored it.");
        }
    }
}

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[Y_BUTTON_INDEX] == 1 && teachingStartTime == ros::Time(0))
    {
        ROS_INFO("Starting Teaching.");
        teachingStartTime = ros::Time::now(); 
    }
}

void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    lastOdomPosition = msg->pose.pose;

    if(teachingStartTime != ros::Time(0))
    {
        *pPositionRecord <<
            boost::lexical_cast<std::string>((ros::Time::now() - teachingStartTime).toSec()) << "," <<
            geo_util::poseToString(lastOdomPosition);
    }
}

void encodersCallback(const clearpath_base::Encoders::ConstPtr& msg)
{
    lastTravelRecorded = msg->encoders[0].travel;
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(teachingStartTime != ros::Time(0))
    {
        *pSpeedRecord <<
            boost::lexical_cast<std::string>(
                    (ros::Time::now() - teachingStartTime).toSec())
            << L_SEP << msg->linear.x << L_SEP << msg->angular.z << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_teach");
    ros::NodeHandle n;

    nextCloudIndex = 0;     
    anchorPointList = std::vector<AnchorPoint>();

    ros::Subscriber sub = n.subscribe(POINT_CLOUD_TOPIC, 10, cloudCallback);
    ros::Subscriber joystickTopic =
        n.subscribe(JOYSTICK_TOPIC, 5000, joystickCallback);
    ros::Subscriber pose_topic =
        n.subscribe(POSE_ESTIMATE_TOPIC, 1000, odomCallback);
    ros::Subscriber encoderTopic =
        n.subscribe(WHEEL_TRAVEL_TOPIC, 1000, encodersCallback);
    ros::Subscriber velTopic =
        n.subscribe(VEL_TOPIC, 1000, velocityCallback);
    ros::ServiceClient crClient = n.serviceClient<pointcloud_tools::CloudRecorder>("cloud_recorder");
    pCrClient = &crClient;
    tf::TransformListener tfListener;

    std::ofstream positionRecord("positions.pl");
    std::ofstream speedRecord("speeds.sl");
    pPositionRecord = &positionRecord;
    pSpeedRecord = &speedRecord;

    // Fetch and store the transformation from the lidar to the base_link, we'll need it when
    // saving the point clouds.
    tLidarToBaseLink  =
            PointMatcher_ros::transformListenerToEigenMatrix<float>(tfListener, ROBOT_FRAME,
                                                                    LIDAR_FRAME, ros::Time(0));

    lastTravelRecorded = 0.0;
    travelOfLastAnchor = 0.0;
    teachingStartTime = ros::Time(0);

    ros::Rate loop_rate(LOOP_RATE);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    positionRecord.close();
    speedRecord.close();
    saveAnchorPointList(anchorPointList);

    return 0;
}
