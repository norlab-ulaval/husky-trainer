
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/iterator.hpp>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

#include "husky_trainer/Repeat.h"
#include "husky_trainer/ControllerMappings.h"

// Parameter names.
const std::string Repeat::SOURCE_TOPIC_PARAM = "readings_topic";
const std::string Repeat::COMMAND_OUTPUT_PARAM = "command_output_topic";
const std::string Repeat::WORKING_DIRECTORY_PARAM = "working_directory";

// Default values.
const std::string Repeat::DEFAULT_SOURCE_TOPIC = "/cloud";
const std::string Repeat::DEFAULT_COMMAND_OUTPUT_TOPIC =
        "/teach_repeat/desired_command";

const double Repeat::LOOP_RATE = 100.0;
const std::string Repeat::JOY_TOPIC = "/joy";
const std::string Repeat::REFERENCE_POSE_TOPIC = "/teach_repeat/reference_pose";
const std::string Repeat::ERROR_REPORTING_TOPIC = "/teach_repeat/raw_error";
const std::string Repeat::AP_SWITCH_TOPIC = "/teach_repeat/ap_switch";
const std::string Repeat::CLOUD_MATCHING_SERVICE = "/match_clouds";
const std::string Repeat::LIDAR_FRAME = "/velodyne";
const std::string Repeat::ROBOT_FRAME = "/base_link";
const std::string Repeat::WORLD_FRAME = "/odom";

Repeat::Repeat(ros::NodeHandle n) :
    loopRate(LOOP_RATE), controller(n)
{
    std::string workingDirectory;

    // Read parameters.
    n.param<std::string>(SOURCE_TOPIC_PARAM, sourceTopicName, DEFAULT_SOURCE_TOPIC);
    n.param<std::string>(WORKING_DIRECTORY_PARAM, workingDirectory, "");

    if(!chdir(workingDirectory.c_str()) != 0)
    {
        ROS_WARN("Failed to switch to the demanded directory.");
    }

    // Read from the teach files.
    loadCommands("speeds.sl", commands);
    loadPositions("positions.pl", positions);
    loadAnchorPoints("anchorPoints.apd", anchorPoints);
    ROS_INFO_STREAM("Done loading the teach in memory.");

    currentStatus = PAUSE;
    commandCursor = commands.begin();
    positionCursor = positions.begin();
    anchorPointCursor = anchorPoints.begin();

    // Make the appropriate subscriptions.
    readingTopic = n.subscribe(sourceTopicName, 10, &Repeat::cloudCallback, this);
    joystickTopic = n.subscribe(JOY_TOPIC, 1000, &Repeat::joystickCallback, this);
    errorReportingTopic = n.advertise<husky_trainer::TrajectoryError>(ERROR_REPORTING_TOPIC, 1000);
    commandRepeaterTopic = n.advertise<geometry_msgs::Twist>(DEFAULT_COMMAND_OUTPUT_TOPIC, 1000);
    referencePoseTopic = n.advertise<geometry_msgs::Pose>(REFERENCE_POSE_TOPIC, 100);
    anchorPointSwitchTopic = n.advertise<husky_trainer::AnchorPointSwitch>(AP_SWITCH_TOPIC, 1000);

    icpService = n.serviceClient<pointmatcher_ros::MatchClouds>(CLOUD_MATCHING_SERVICE, false);

    // Fetch the transform from lidar to base_link and cache it.
    tf::TransformListener tfListener;
    tfListener.waitForTransform(ROBOT_FRAME, LIDAR_FRAME, ros::Time(0), ros::Duration(5.0));
    tfListener.lookupTransform(ROBOT_FRAME, LIDAR_FRAME, ros::Time(0), tFromLidarToRobot);

    // Setup the dynamic reconfiguration server.
    dynamic_reconfigure::Server<husky_trainer::RepeatConfig>::CallbackType callback;
    callback = boost::bind(&Repeat::paramCallback, this, _1, _2);
    drServer.setCallback(callback);
}

void Repeat::spin()
{
    while(ros::ok())
    {
        ros::Time timeOfSpin = simTime();
        updateAnchorPoint();

        //Update the command we are playing.
        if(currentStatus == FORWARD || currentStatus == REWIND)
        {
            geometry_msgs::Twist nextCommand =
                controller.correctCommand(commandOfTime(timeOfSpin));
            commandRepeaterTopic.publish(nextCommand);
        }

        referencePoseTopic.publish(poseOfTime(simTime()));

        ros::spinOnce();
        loopRate.sleep();
    }
}

Repeat::~Repeat()
{
    readingTopic.shutdown();
    serviceCallLock.lock();
    serviceCallLock.unlock();
}


void Repeat::updateAnchorPoint()
{
    ros::Time timeOfUpdate = simTime();

    double distanceToCurrentAnchorPoint =
            geo_util::customDistance(poseOfTime(timeOfUpdate), anchorPointCursor->getPosition());

    double distanceToNextAnchorPoint;
    if(currentStatus == FORWARD) {
        distanceToNextAnchorPoint =
            anchorPointCursor != anchorPoints.end() - 1 ?
            geo_util::customDistance(
                poseOfTime(timeOfUpdate),
                anchorPointCursor->getPosition()) :
            std::numeric_limits<double>::infinity();
    } else if (currentStatus == REWIND) {
        distanceToNextAnchorPoint =
            anchorPointCursor != anchorPoints.begin() ?
            geo_util::customDistance(
                poseOfTime(timeOfUpdate),
                anchorPointCursor->getPosition()) :
            std::numeric_limits<double>::infinity();
    } else {
        distanceToNextAnchorPoint = std::numeric_limits<double>::infinity();
    }

    //ROS_INFO("Distances. Current: %f, Next: %f", distanceToCurrentAnchorPoint, distanceToNextAnchorPoint);

    // Update the closest anchor point.
    if(boost::next(anchorPointCursor) < anchorPoints.end() &&
        distanceToCurrentAnchorPoint >= distanceToNextAnchorPoint)
    {
        if (currentStatus == FORWARD) anchorPointCursor++;
        else if (currentStatus == REWIND) anchorPointCursor--;
        else ROS_ERROR("Invalid status when updating anchor point");

        husky_trainer::AnchorPointSwitch msg;
        msg.stamp = ros::Time::now();
        msg.newAnchorPoint = anchorPointCursor->name();
        anchorPointSwitchTopic.publish(msg);
    }
}

void Repeat::updateError(const sensor_msgs::PointCloud2& reading)
{
    tf::Transform tFromReadingToAnchor =
            geo_util::transFromPoseToPose(poseOfTime(simTime()), anchorPointCursor->getPosition());

    Eigen::Matrix4f eigenTransform;
    pcl_ros::transformAsMatrix(tFromReadingToAnchor*tFromLidarToRobot, eigenTransform);

    sensor_msgs::PointCloud2 transformedReadingCloudMsg;
    pcl_ros::transformPointCloud(eigenTransform, reading, transformedReadingCloudMsg);

    pointmatcher_ros::MatchClouds pmMessage;
    pmMessage.request.readings = transformedReadingCloudMsg;
    pmMessage.request.reference = anchorPointCursor->getCloud();

    if(serviceCallLock.try_lock())
    {
        if(icpService.call(pmMessage))
        {
            husky_trainer::TrajectoryError rawError =
                pointmatching_tools::controlErrorOfTransformation(
                        pmMessage.response.transform
                    );

            errorReportingTopic.publish(rawError);
            controller.updateError(rawError);
        } else {
            ROS_WARN("There was a problem with the point matching service.");
            switchToStatus(ERROR);
        }
        serviceCallLock.unlock();
    } else {
        ROS_DEBUG("ICP service was busy, dropped a cloud.");
    }
}

void Repeat::cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    boost::thread thread(&Repeat::updateError, this, *msg);
}

void Repeat::joystickCallback(sensor_msgs::Joy::ConstPtr msg)
{
    switch(currentStatus)
    {
    case FORWARD:
        if(msg->buttons[controller_mappings::RB] == 0) switchToStatus(PAUSE);
        break;
    case REWIND:
        if(msg->buttons[controller_mappings::RT] == 0) switchToStatus(PAUSE);
        break;
    case PAUSE:
        if(msg->buttons[controller_mappings::RB] == 1) {
            switchToStatus(FORWARD);
        } else if(msg->buttons[controller_mappings::RT] == 1) {
            switchToStatus(REWIND);
        }
        break;
    case ERROR:
        if(msg->buttons[controller_mappings::X] == 1) switchToStatus(PAUSE);
        break;
    }
}

geometry_msgs::Twist Repeat::commandOfTime(ros::Time time)
{
    geometry_msgs::Twist output;

    if(currentStatus == FORWARD) {
        while(commandCursor->header.stamp < time + ros::Duration(lookahead) &&
                commandCursor < commands.end() - 1) {
           commandCursor++;
        }

        output = commandCursor->twist;
    } else if (currentStatus == REWIND) {
        ros::Time lookaheadAdjustedTime = trySubstract(ros::Duration(lookahead), time);

        while(commandCursor->header.stamp >= lookaheadAdjustedTime &&
                commandCursor > commands.begin()) {
            commandCursor--;
        }

        output = reverseCommand(commandCursor->twist);
    } else {
        output = CommandRepeater::idleTwistCommand();
    }
    return output;
}

geometry_msgs::Pose Repeat::poseOfTime(ros::Time time)
{
    if(currentStatus == FORWARD) {
        while(positionCursor->header.stamp < time + ros::Duration(lookahead) &&
                positionCursor < positions.end() - 1) {
            positionCursor++;
        }
    } else if (currentStatus == REWIND) {
        while(positionCursor->header.stamp >= trySubstract(ros::Duration(lookahead), time)&&
                positionCursor > positions.begin()) {
            positionCursor--;
        }
    }
    return positionCursor->pose;
}

void Repeat::pausePlayback()
{
    commandRepeaterTopic.publish(CommandRepeater::idleTwistCommand());

    if(currentStatus == FORWARD) {
        baseSimTime += ros::Time::now() - timePlaybackStarted;
    } else if (currentStatus == REWIND) {
        ros::Duration delta = ros::Time::now() - timePlaybackStarted;

        baseSimTime = trySubstract(delta, baseSimTime);
    }

    ROS_INFO("Paused at: %lf", baseSimTime.toSec());
}

void Repeat::startPlayback()
{
    timePlaybackStarted = ros::Time::now();
}

ros::Time Repeat::simTime()
{
    ros::Time simTime;
    ros::Duration delta = ros::Time::now() - timePlaybackStarted;

    if(currentStatus == FORWARD) {
        simTime = baseSimTime + delta;
    } else if (currentStatus == REWIND)  {
        // We can't compare a Duration to a Time, and it seems risky to ever
        // create a negative Time, so we add the delta to ros::Time(0) to
        // compare it with baseSimTime.
        simTime = trySubstract(delta, baseSimTime);
    } else {
        simTime = baseSimTime;
    }

    return simTime;
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
    case FORWARD:
        if(desiredStatus == PAUSE)
        {
            ROS_INFO("Stopping playback.");
            pausePlayback();
            currentStatus = PAUSE;
        }
        break;

    case REWIND:
        if(desiredStatus == PAUSE)
        {
            ROS_INFO("Stopping playback.");
            pausePlayback();
            currentStatus = PAUSE;
        }
        break;

    case PAUSE:
        if(desiredStatus == FORWARD || desiredStatus == REWIND)
        {
            ROS_INFO("Starting playback.");
            startPlayback();
            currentStatus = desiredStatus;
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

void Repeat::paramCallback(husky_trainer::RepeatConfig& params, uint32_t level)
{
    lookahead = ros::Duration(params.lookahead);
    controller.updateParams(params);
}

geometry_msgs::Twist Repeat::reverseCommand(geometry_msgs::Twist input)
{
    geometry_msgs::Twist output;
    output.linear.x = -1.0 * input.linear.x;
    output.angular.z = -1.0 * input.angular.z;

    return output;
}

// Try to substract a value from a time without going into negative times.
ros::Time Repeat::trySubstract(ros::Duration value, ros::Time from)
{
    return ros::Time(0.0) + value < from ?
                from - value:
                ros::Time(0.0);
}
