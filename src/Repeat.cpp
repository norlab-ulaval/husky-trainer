
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/iterator.hpp>

#include <pcl_ros/point_cloud.h>

#include "husky_trainer/Repeat.h"
#include "husky_trainer/ControllerMappings.h"

// Parameter names.
const std::string Repeat::LAMBDA_X_PARAM = "lx";
const std::string Repeat::LAMBDA_Y_PARAM = "ly";
const std::string Repeat::LAMBDA_THETA_PARAM = "lt";
const std::string Repeat::LOOKAHEAD_PARAM = "lookahead";
const std::string Repeat::SOURCE_TOPIC_PARAM = "readings_topic";
const std::string Repeat::COMMAND_OUTPUT_PARAM = "command_output_topic";
const std::string Repeat::WORKING_DIRECTORY_PARAM = "working_directory";

// Default values.
const double Repeat::DEFAULT_LAMBDA_X = 0.0;
const double Repeat::DEFAULT_LAMBDA_Y = 0.0;
const double Repeat::DEFAULT_LAMBDA_THETA = 0.0;
const double Repeat::DEFAULT_LOOKAHEAD = 0.2;
const std::string Repeat::DEFAULT_SOURCE_TOPIC = "/cloud";
const std::string Repeat::DEFAULT_COMMAND_OUTPUT_TOPIC = "/teach_repeat/desired_command";

const double Repeat::LOOP_RATE = 100.0;
const std::string Repeat::JOY_TOPIC = "/joy";
const std::string Repeat::CLOUD_MATCHING_SERVICE = "/match_clouds";
const std::string Repeat::LIDAR_FRAME = "/velodyne";
const std::string Repeat::ROBOT_FRAME = "/base_footprint";
const std::string Repeat::WORLD_FRAME = "/odom";

Repeat::Repeat(ros::NodeHandle n) :
    loopRate(LOOP_RATE)
{
    std::string workingDirectory;

    // Read parameters.
    n.param<double>(LAMBDA_X_PARAM, lambdaX, DEFAULT_LAMBDA_X);
    n.param<double>(LAMBDA_Y_PARAM, lambdaY, DEFAULT_LAMBDA_Y);
    n.param<double>(LAMBDA_THETA_PARAM, lambdaTheta, DEFAULT_LAMBDA_THETA);
    n.param<double>(LOOKAHEAD_PARAM, lookahead, DEFAULT_LOOKAHEAD);
    n.param<std::string>(SOURCE_TOPIC_PARAM, sourceTopicName, DEFAULT_SOURCE_TOPIC);
    n.param<std::string>(WORKING_DIRECTORY_PARAM, workingDirectory, "");

    if(!chdir(workingDirectory.c_str()) != 0)
    {
        ROS_WARN("Failed to switch to the demanded directory.");
    }

    ROS_INFO_STREAM("Lambda values: " << lambdaX << ", " << lambdaY << ", " << lambdaTheta << ".");
    ROS_INFO_STREAM("Lookahead: " << lookahead);

    // Read from the teach files.
    loadCommands("speeds.sl", commands);
    loadPositions("positions.pl", positions);
    loadAnchorPoints("anchorPoints.apd", anchorPoints);
    ROS_INFO_STREAM("Done loading the teach in memory.");

    currentStatus = PAUSE;
    currentError = IcpError(0.0,0.0,0.0);
    commandCursor = commands.begin();
    positionCursor = positions.begin();
    anchorPointCursor = anchorPoints.begin();

    // Make the appropriate subscriptions.
    readingTopic = n.subscribe(sourceTopicName, 10, &Repeat::cloudCallback, this);
    joystickTopic = n.subscribe(JOY_TOPIC, 1000, &Repeat::joystickCallback, this);
    commandRepeaterTopic = n.advertise<geometry_msgs::Twist>(DEFAULT_COMMAND_OUTPUT_TOPIC, 1000);
    icpService = n.serviceClient<pointmatcher_ros::MatchClouds>(CLOUD_MATCHING_SERVICE, false);

    // Fetch the transform from lidar to base_link and cache it.
    tf::TransformListener tfListener;
    tFromLidarToRobot =
            PointMatcher_ros::transformListenerToEigenMatrix<float>(tfListener, ROBOT_FRAME,
                                                                    LIDAR_FRAME, ros::Time(0));
}

void Repeat::spin()
{
    while(ros::ok() && commandCursor != commands.end())
    {
        ros::Time timeOfSpin = simTime();

        // Update the closest anchor point.
        if(boost::next(anchorPointCursor) < anchorPoints.end() &&
                geo_util::customDistance(poseOfTime(timeOfSpin),
                                         anchorPointCursor->getPosition()) >
                geo_util::customDistance(poseOfTime(timeOfSpin),
                                         boost::next(anchorPointCursor)->getPosition()))
        {
            anchorPointCursor++;
        }

        //Update the command we are playing.
        commandRepeaterTopic.publish(errorAdjustedCommand(commandOfTime(timeOfSpin), currentError));

        ros::spinOnce();
        loopRate.sleep();
    }
}

void Repeat::updateError(const sensor_msgs::PointCloud2& reading)
{
    ros::Time timeOfUpdate = ros::Time::now();

    PM::TransformationParameters tFromReadingToAnchor =
            geo_util::transFromPoseToPose(poseOfTime(timeOfUpdate), anchorPointCursor->getPosition());

    sensor_msgs::PointCloud2 transformedReadingCloudMsg =
            pointmatching_tools::applyTransform(reading, tFromReadingToAnchor*tFromLidarToRobot);

    sensor_msgs::PointCloud2 referenceCloudMsg =
            PointMatcher_ros::pointMatcherCloudToRosMsg<float>(anchorPointCursor->getCloud(), WORLD_FRAME, ros::Time(0));

    // Service call.
    pointmatcher_ros::MatchClouds pmMessage;
    pmMessage.request.readings = transformedReadingCloudMsg;
    pmMessage.request.reference = referenceCloudMsg;

    if(serviceCallLock.try_lock())
    {
        if(icpService.call(pmMessage))
        {
            currentError = pointmatching_tools::controlErrorOfTransformation(pmMessage.response.transform);
            ROS_INFO("Error. X: %f, Y: %f, Theta: %f", currentError.get<0>(), currentError.get<1>(), currentError.get<2>());
        } else {
            ROS_WARN("There was a problem with the point matching service.");
            switchToStatus(ERROR);
        }
        serviceCallLock.unlock();
    } else {
        ROS_INFO("ICP service was busy, dropped a cloud.");
    }
}

geometry_msgs::Twist Repeat::errorAdjustedCommand(geometry_msgs::Twist originalCommand, IcpError error)
{
    float newAngular = originalCommand.angular.z + lambdaY * error.get<1>() - lambdaTheta * error.get<2>();
    float newLinear = originalCommand.linear.x * cos(error.get<1>()) - lambdaX * error.get<0>();

    originalCommand.angular.z = newAngular;
    originalCommand.linear.x = newLinear;

    return originalCommand;
}

void Repeat::cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    boost::thread thread(&Repeat::updateError, this, *msg);
}

void Repeat::joystickCallback(sensor_msgs::Joy::ConstPtr msg)
{
    switch(currentStatus)
    {
    case PLAY:
        if(msg->buttons[controller_mappings::RB] == 0) switchToStatus(PAUSE);
        break;
    case PAUSE:
        if(msg->buttons[controller_mappings::RB] == 1) switchToStatus(PLAY);
        break;
    case ERROR:
        if(msg->buttons[controller_mappings::X] == 1) switchToStatus(PAUSE);
        break;
    }
}

geometry_msgs::Twist Repeat::commandOfTime(ros::Time time)
{
    std::vector<geometry_msgs::TwistStamped>::iterator previousCursor = commandCursor;

    while(commandCursor->header.stamp < time && commandCursor < commands.end() - 1)
    {
        previousCursor = commandCursor++;
    }

    return commandCursor->twist;
}

geometry_msgs::Pose Repeat::poseOfTime(ros::Time time)
{
    std::vector<geometry_msgs::PoseStamped>::iterator previousCursor = positionCursor;

    while(positionCursor->header.stamp < time && positionCursor < positions.end() - 1)
    {
        previousCursor = positionCursor++;
    }

    return positionCursor->pose;
}

void Repeat::pausePlayback()
{
    commandRepeaterTopic.publish(CommandRepeater::idleTwistCommand());
    baseSimTime += ros::Time::now() - timePlaybackStarted;
}

void Repeat::startPlayback()
{
    timePlaybackStarted = ros::Time::now();
}

ros::Time Repeat::simTime()
{
    if(currentStatus == PLAY) return baseSimTime + (ros::Time::now() - timePlaybackStarted);
    else return baseSimTime;
}

void Repeat::switchToStatus(Status desiredStatus)
{
    if(desiredStatus == ERROR && currentStatus != ERROR)
    {
        ROS_WARN("Switching to emergency mode.");
        pausePlayback();
        currentStatus = ERROR;
    }

    switch(currentStatus)
    {
    case PLAY:
        if(desiredStatus == PAUSE)
        {
            ROS_INFO("Stopping playback.");
            pausePlayback();
            currentStatus = PAUSE;
        }
        break;
    case PAUSE:
        if(desiredStatus == PLAY)
        {
            ROS_INFO("Starting playback.");
            startPlayback();
            currentStatus = PLAY;
        }
        break;
    case ERROR:
        if(desiredStatus == PAUSE)
        {
            ROS_INFO("Attempting recovery.");
            currentStatus = PAUSE;
        }
        break;
    }
}

void Repeat::loadCommands(const std::string filename, std::vector<geometry_msgs::TwistStamped>& out)
{
    std::ifstream commandFile(filename.c_str());
    out.clear();

    if(commandFile.is_open())
    {
        std::string lineBuffer;
        while(std::getline(commandFile, lineBuffer))
        {
            out.push_back(geo_util::stampedTwistOfString(lineBuffer));
        }

        commandFile.close();
    }
    else {
        ROS_ERROR_STREAM("Could not open command file: " << filename);
    }
}

void Repeat::loadPositions(const std::string filename, std::vector<geometry_msgs::PoseStamped>& out)
{
    std::ifstream positionFile(filename.c_str());
    out.clear();

    if(positionFile.is_open())
    {
        std::string lineBuffer;
        while(std::getline(positionFile, lineBuffer))
        {
            out.push_back(geo_util::stampedPoseOfString(lineBuffer));
        }

        positionFile.close();
    } else {
        ROS_ERROR_STREAM("Could not open position file: " << filename);
    }
}

void Repeat::loadAnchorPoints(const std::string filename, std::vector<AnchorPoint>& out)
{
    std::ifstream anchorPointsFile(filename.c_str());

    if(anchorPointsFile.is_open())
    {
        std::string lineBuffer;
        while(std::getline(anchorPointsFile, lineBuffer))
        {
            out.push_back(AnchorPoint(lineBuffer));
            out.back().loadFromDisk();
        }
        anchorPointsFile.close();
    } else {
        ROS_ERROR_STREAM("Could not open anchor points file: " << filename);
    }
}
