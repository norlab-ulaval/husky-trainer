
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
#include "pointmatcher_ros/MatchClouds.h"

#define JOY_TOPIC "/joy"
#define CMD_TOPIC "/teach_repeat/desired_command"
#define WORLD_FRAME "/odom"
#define ROBOT_FRAME "/base_footprint"
#define LIDAR_FRAME "/velodyne"
#define CLOUD_MATCHING_SERVICE "/match_clouds"

#define POS_FILE_DELIMITER ','
#define Y_BUTTON_INDEX 3
#define DM_SWITCH_INDEX 5 // Deadman switch index on the gamepad. It has to be a different button
                          // than the usual deadman switch, so that the joy stick does not send
                          // "don't" move commands on top of the repeat commands.
#define LOOP_RATE 100

#define LAMBDA_X_PARAM "lx"
#define LAMBDA_Y_PARAM "ly"
#define LAMBDA_T_PARAM "lt"
#define LOOKAHEAD_PARAM "lookahead"
#define SOURCE_PARAM "source"

// TODO: Turn those next parameters into variables and interface them to be changed easily.
#define DEFAULT_LAMBDA_X 0.0
#define DEFAULT_LAMBDA_Y 0.0
#define DEFAULT_LAMBDA_THETA 0.0
#define DEFAULT_SPEED_LOOKAHEAD 0.2
#define DEFAULT_SOURCE_PARAM "/cloud"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

// --- Node-global variables definition ---
ros::Time repeatBeginTime;
ros::Time simTime;  // How far we are in the teach playback.
ros::Time timeLastCommandWasRead; // How long have we been playing the last command.
bool playbackIsOn = false;
int closestAnchorIndex;
std::vector<AnchorPoint> anchorPoints;
std::string sourceTopic;

PM::TransformationParameters tFromLidarToBaseLink;
boost::mutex currentErrorMutex;
ControlError currentError;
ros::ServiceClient* pPmService;
ros::Publisher* pCmd;
std::vector< boost::tuple<double, geometry_msgs::Pose> > positionList;
std::vector< boost::tuple<double, geometry_msgs::Pose> >::iterator positionBegin;
std::vector< boost::tuple<double, geometry_msgs::Pose> >::iterator positionEnd;

double lambdaX;
double lambdaY;
double lambdaTheta;
double lookahead;

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

    if(begin == end)
    {
        ROS_INFO("Reached the end of the command feed.");
        return (--begin)->get<1>();
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

    commandFile.close();
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

    positionFile.close();
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

    anchorPointsFile.close();
    return retVal;
}

geometry_msgs::Twist errorAdjustedCommand(geometry_msgs::Twist originalCommand, ControlError error)
{   
    float newAngular = originalCommand.angular.z + lambdaY * error.get<1>() - lambdaTheta * error.get<2>();
    float newLinear = originalCommand.linear.x * cos(error.get<1>()) - lambdaX * error.get<0>();

    originalCommand.angular.z = newAngular;
    originalCommand.linear.x = newLinear;

    return originalCommand;
}

void updateError(const sensor_msgs::PointCloud2& msg)
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

    ros::Time begin = ros::Time::now();
    sensor_msgs::PointCloud2 transformedMsg = applyTransform(msg, tFromReadingToAnchor*tFromLidarToBaseLink);
    ROS_INFO("applyTransform: %lf", (ros::Time::now() - begin).toSec());

    pointmatcher_ros::MatchClouds pmMessage;
    pmMessage.request.readings = transformedMsg;
    pmMessage.request.reference = referenceMsg;

    begin = ros::Time::now();
    if(pPmService->call(pmMessage))
    {
        ROS_INFO("service call: %lf", (ros::Time::now() - begin).toSec());
        ControlError newError = controlErrorOfTransformation(pmMessage.response.transform);

        currentErrorMutex.lock();
        currentError = newError;
        currentErrorMutex.unlock();
        ROS_INFO("Error. X: %f, Y: %f, Theta: %f", newError.get<0>(), newError.get<1>(), newError.get<2>());
    }
    else
    {
        ROS_WARN("There was a problem with the point matching service.");
    }
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    boost::thread thread(updateError, *msg);
}

void joystickCallback(sensor_msgs::Joy::ConstPtr msg)
{
    if(msg->buttons[DM_SWITCH_INDEX] == 1 && !playbackIsOn)
    {
        ROS_INFO("Starting playback.");
        playbackIsOn = true;
        timeLastCommandWasRead = ros::Time::now();
    }
    else if(msg->buttons[DM_SWITCH_INDEX] == 0 && playbackIsOn)
    {
        ROS_INFO("Stopping playback.");
        playbackIsOn = false;

        geometry_msgs::Twist stopMoving; // It's init at 0 by default.
        pCmd->publish(stopMoving);
    }
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "husky_repeat");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(LOOP_RATE);

    playbackIsOn = false;
    simTime = ros::Time(0);

    n.param<double>(LAMBDA_X_PARAM, lambdaX, DEFAULT_LAMBDA_X);
    n.param<double>(LAMBDA_Y_PARAM, lambdaY, DEFAULT_LAMBDA_Y);
    n.param<double>(LAMBDA_T_PARAM, lambdaTheta, DEFAULT_LAMBDA_THETA);
    n.param<double>(LOOKAHEAD_PARAM, lookahead, DEFAULT_SPEED_LOOKAHEAD);
    n.param<std::string>(SOURCE_PARAM, sourceTopic, DEFAULT_SOURCE_PARAM);

    ros::Subscriber cloud = n.subscribe(sourceTopic, 10, cloudCallback);
    ros::Subscriber joystick = n.subscribe(JOY_TOPIC, 5000, joystickCallback);
    ros::Publisher cmd = n.advertise<geometry_msgs::Twist>(CMD_TOPIC,1000);
    pCmd = &cmd;
    ros::ServiceClient pmClient = n.serviceClient<pointmatcher_ros::MatchClouds>(CLOUD_MATCHING_SERVICE, true);
    pPmService = &pmClient;
    tf::TransformListener tfListener;

    ROS_INFO_STREAM("Lambda values: " << lambdaX << ", " << lambdaY << ", " << lambdaTheta << ".");

    positionList = loadPositions();
    positionBegin = positionList.begin();
    positionEnd = positionList.end();
    ROS_INFO("Loaded positions.");

    std::vector< boost::tuple<double, geometry_msgs::Twist> > commandList = loadCommands();
    ROS_INFO("Loaded command feed.");
    int nextCommand = 0;

    closestAnchorIndex = 0;
    anchorPoints = loadAnchorPoints();
    ROS_INFO("Loaded anchor points");

    currentError = ControlError(0.0,0.0,0.0);

    // Fetch and store the transformation from the lidar to the base_link, we'll need it when
    // comparing the points clouds.
    tFromLidarToBaseLink  =
            PointMatcher_ros::transformListenerToEigenMatrix<float>(tfListener, ROBOT_FRAME,
                                                                    LIDAR_FRAME, ros::Time(0));
    ros::Time nextCommandTime;
    geometry_msgs::Pose posOfTime;

    while(ros::ok() && nextCommand < commandList.size())
    {
        if(playbackIsOn)
        {
            nextCommandTime.fromSec(commandList[nextCommand].get<0>());
            ros::Duration delta = (ros::Time::now() - timeLastCommandWasRead);

            // If we have been playing the last command as long as it was played during the teach,
            // switch to the next command.
            if(delta > (nextCommandTime - simTime))
            {
                simTime.fromSec(commandList[nextCommand].get<0>());
                timeLastCommandWasRead = ros::Time::now();
                cmd.publish( errorAdjustedCommand( commandList[nextCommand++].get<1>(), currentError) );

                // Update the reference position.
                posOfTime = positionOfTime(simTime, positionBegin, positionEnd);
            } else {
                cmd.publish( errorAdjustedCommand(commandList[nextCommand].get<1>(), currentError));
            }

            if(closestAnchorIndex != anchorPoints.size() - 1 &&
                    geo_util::customDistance(posOfTime, anchorPoints[closestAnchorIndex].getPosition()) >=
                    geo_util::customDistance(posOfTime, anchorPoints[closestAnchorIndex + 1].getPosition()))
            {
                closestAnchorIndex++;
                ROS_INFO("Swapping to anchor point number: %d", closestAnchorIndex);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    return 0;
}
