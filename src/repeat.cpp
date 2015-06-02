
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <math.h>

#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>

#include "husky_trainer/GeoUtil.h"
#include "husky_trainer/AnchorPoint.h"
#include "husky_trainer/PointMatching.h"
#include "husky_trainer/MatchClouds.h"

#define JOY_TOPIC "/joy"
#define CLOUD_TOPIC "/cloud"
#define CMD_TOPIC "/husky/cmd_vel"
#define WORLD_FRAME "/odom"
#define ROBOT_FRAME "/base_footprint"
#define LIDAR_FRAME "/velodyne"

#define POS_FILE_DELIMITER ','
#define ICP_CONFIG_FILE "config.yaml"
#define X_BUTTON_INDEX 2
#define LOOP_RATE 100

// TODO: Turn those next parameters into variables and interface them to be changed easily.
#define LAMBDA_X 0.6
#define LAMBDA_Y 14.0
#define LAMBDA_THETA 3.5
#define SPEED_LOOKAHEAD 0.2

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

// --- Node-global variables definition ---
ros::Time repeatBeginTime;
bool repeatBegan = false;
int closestAnchorIndex;
std::vector<AnchorPoint> anchorPoints;
ros::Time simTime;  // How far we are in the teach playback.
PM::TransformationParameters tFromLidarToBaseLink;
boost::mutex currentErrorMutex;
ControlError currentError;
ros::ServiceClient* pPmService;
std::vector< boost::tuple<double, geometry_msgs::Pose> > positionList;
std::vector< boost::tuple<double, geometry_msgs::Pose> >::iterator positionBegin;
std::vector< boost::tuple<double, geometry_msgs::Pose> >::iterator positionEnd;


int closestAnchor(const std::vector<AnchorPoint>& anchorPointList,
                  const geometry_msgs::Pose position)
{
    geometry_msgs::Pose firstPose = anchorPointList[0].getPosition();
    double minDistance = geo_util::customDistance(firstPose, position);
    int closestSoFar = 0;

    for(int i=1; i < anchorPointList.size(); i++)
    {
        geometry_msgs::Pose anchorPose = anchorPointList[i].getPosition();

        double distance = geo_util::customDistance(anchorPose, position);
        if(distance < minDistance || minDistance < 0)
        {
            minDistance = distance;
            closestSoFar = i;
        }
    }

    ROS_INFO("Distance from closest anchor point: %lf", minDistance);

    return closestSoFar;
}

geometry_msgs::Pose positionOfTime(ros::Time time,
        std::vector< boost::tuple<double, geometry_msgs::Pose> >::iterator& begin, 
        std::vector< boost::tuple<double, geometry_msgs::Pose> >::iterator& end)
{
    while(begin != end && begin->get<0>() < time.toSec())
    {
        begin++;
    }

    return begin->get<1>();
}


boost::tuple<double, geometry_msgs::Twist> commandOfLine(std::string line)     
{
    std::stringstream ss(line);
    std::string buffer;

    std::getline(ss,buffer,POS_FILE_DELIMITER);
    double time = strtod(buffer.c_str(), NULL);

    std::getline(ss,buffer,POS_FILE_DELIMITER);
    double linear = strtod(buffer.c_str(), NULL);

    std::getline(ss,buffer,POS_FILE_DELIMITER);
    double angular = strtod(buffer.c_str(), NULL);

    geometry_msgs::Vector3 linearSpeed;
    linearSpeed.x = linear;
    linearSpeed.y = 0.0;
    linearSpeed.z = 0.0;

    geometry_msgs::Vector3 angularSpeed;
    angularSpeed.x = 0.0;
    angularSpeed.y = 0.0;
    angularSpeed.z = angular;

    geometry_msgs::Twist msg;
    msg.linear = linearSpeed;
    msg.angular = angularSpeed;

    return boost::tuple<double, geometry_msgs::Twist>(time, msg);
}

boost::tuple<double,geometry_msgs::Pose> coordinatesOfLine(std::string line)
{
    std::stringstream ss(line);
    std::string buffer;

    std::getline(ss,buffer,POS_FILE_DELIMITER);
    double time = strtod(buffer.c_str(), NULL);

    geometry_msgs::Pose pose = geo_util::stringToPose(ss.str());

    return boost::tuple<double,geometry_msgs::Pose>(time,pose);
}

std::vector< boost::tuple<double,geometry_msgs::Twist> > loadCommands()
{
    std::vector< boost::tuple<double,geometry_msgs::Twist> > retVal = 
        std::vector< boost::tuple<double,geometry_msgs::Twist> >();

    std::ifstream commandFile("speeds.sl");

    if(commandFile.is_open())
    {
        std::string lineBuffer;
        while(std::getline(commandFile,lineBuffer))
        {
            boost::tuple<double,geometry_msgs::Twist> command = 
                commandOfLine(lineBuffer);

            retVal.push_back(command);
        }
    }

    return retVal;
}

std::vector< boost::tuple<double,geometry_msgs::Pose> > loadPositions()
{
    std::vector< boost::tuple<double,geometry_msgs::Pose> > retVal = 
        std::vector< boost::tuple<double,geometry_msgs::Pose> >();

    std::ifstream positionFile("positions.pl");

    if(positionFile.is_open())
    {
        std::string lineBuffer;
        while(std::getline(positionFile, lineBuffer))
        {
            std::stringstream ss(lineBuffer);
            std::string timeString;
            std::getline(ss, timeString, ',');
            double time = strtod(timeString.c_str(), NULL);

            std::getline(ss, lineBuffer);

            retVal.push_back(
                    boost::tuple<double, geometry_msgs::Pose>(time, 
                        geo_util::stringToPose(lineBuffer)));
        }
    }   

    return retVal;
}

std::vector< AnchorPoint > loadAnchorPoints()
{
    std::vector< AnchorPoint > retVal;
    std::ifstream anchorPointsFile("anchorPoints.apd");

    std::string lineBuffer;
    while(std::getline(anchorPointsFile, lineBuffer))
    {
        retVal.push_back(AnchorPoint(lineBuffer));
        retVal.back().loadFromDisk();
    }

    return retVal;
}

geometry_msgs::Twist errorAdjustedCommand(geometry_msgs::Twist originalCommand, ControlError error)
{
    originalCommand.angular.z =
            originalCommand.angular.z - LAMBDA_Y * error.get<1>() - LAMBDA_THETA * error.get<2>();
    originalCommand.linear.x =
            originalCommand.linear.x * cos(error.get<1>() - LAMBDA_X * error.get<0>());

    return originalCommand;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    // If no anchor points are defined, we have no use for a point cloud.
    if(anchorPoints.size() == 0)
    {
        return;
    }

    DP referenceCloud = anchorPoints[closestAnchorIndex].getCloud();
    sensor_msgs::PointCloud2 referenceMsg =
            PointMatcher_ros::pointMatcherCloudToRosMsg<float>(referenceCloud, WORLD_FRAME, ros::Time(0));

    // Compute the difference between the anchor point and the actual pose and turn it into a
    // transformation matrix.
    PM::TransformationParameters tFromReadingToAnchor = geo_util::transFromPoseToPose(
                positionOfTime(simTime, positionBegin, positionEnd),
                anchorPoints[closestAnchorIndex].getPosition());

    sensor_msgs::PointCloud2 transformedMsg = applyTransform(msg, tFromReadingToAnchor*tFromLidarToBaseLink);

    husky_trainer::MatchClouds pmMessage;
    pmMessage.request.readings = transformedMsg;
    pmMessage.request.reference = referenceMsg;

    if(pPmService->call(pmMessage))
    {
        ControlError newError = controlErrorOfTransformation(pmMessage.response.transform);

        while(currentErrorMutex.try_lock()) {}
        currentError = newError;
        currentErrorMutex.unlock();
        ROS_INFO("Error. X: %f, Y: %f, Theta: %f", newError.get<0>(), newError.get<1>(), newError.get<2>());
    }
    else
    {
        ROS_WARN("There was a problem with the point matching service.");
    }
}

void joystickCallback(sensor_msgs::Joy::ConstPtr msg)
{
    if(msg->buttons[X_BUTTON_INDEX] == 1)
    {
        repeatBeginTime = ros::Time::now();
        repeatBegan = true;
    }
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "husky_repeat");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);

    int nextCommand;

    ros::Subscriber cloud = n.subscribe(CLOUD_TOPIC, 100, cloudCallback);
    ros::Subscriber joystick = n.subscribe(JOY_TOPIC, 5000, joystickCallback);
    ros::Publisher cmd = n.advertise<geometry_msgs::Twist>(CMD_TOPIC,1000);
    ros::ServiceClient pmClient = n.serviceClient<husky_trainer::MatchClouds>("match_clouds");
    pPmService = &pmClient;
    tf::TransformListener tfListener;

    // Fetch and store the transformation from the lidar to the base_link, we'll need it when
    // comparing the points clouds.
    tFromLidarToBaseLink  =
            PointMatcher_ros::transformListenerToEigenMatrix<float>(tfListener, ROBOT_FRAME,
                                                                    LIDAR_FRAME, ros::Time(0));

    positionList = loadPositions();
    positionBegin = positionList.begin();
    positionEnd = positionList.end();
    ROS_INFO("Loaded positions.");

    std::vector< boost::tuple<double, geometry_msgs::Twist> > commandList = loadCommands();
    ROS_INFO("Loaded command feed.");
    nextCommand = 0;

    closestAnchorIndex = 0;
    anchorPoints = loadAnchorPoints();
    ROS_INFO("Loaded anchor points");

    repeatBeginTime = ros::Time::now();
    currentError = ControlError(0.0,0.0,0.0);

    ros::Duration lookaheadAdjustedTime;
    ros::Duration nextCommandTime;

    ROS_INFO("Starting Playback");

    while(ros::ok() && nextCommand < commandList.size())
    {
        lookaheadAdjustedTime.fromSec(SPEED_LOOKAHEAD);
        lookaheadAdjustedTime += ros::Time::now() - repeatBeginTime;

        nextCommandTime.fromSec(commandList[nextCommand].get<0>());

        if(lookaheadAdjustedTime > nextCommandTime)
        {
            simTime.fromSec(commandList[nextCommand].get<0>());
            cmd.publish( errorAdjustedCommand( commandList[nextCommand++].get<1>(), currentError) );
        }

        // Update the reference position
        geometry_msgs::Pose posOfTime = positionOfTime(simTime, positionBegin, positionEnd);

        if(count % 100 == 0)
        {
            //ROS_INFO("T: %lf, X: %lf, Y: %lf", simTime.toSec(), posOfTime.position.x, posOfTime.position.y);

            if(anchorPoints.size() > 0)
            {
                closestAnchorIndex = closestAnchor(anchorPoints, posOfTime);
                ROS_INFO("Closest anchor: %d", closestAnchorIndex);
            } else {
                ROS_INFO("No anchor point found.");
            }
        }
        
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    return 0;
}
