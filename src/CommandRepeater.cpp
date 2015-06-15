
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "husky_trainer/CommandRepeater.h"

CommandRepeater::CommandRepeater(ros::NodeHandle n) :
    desiredCommand()
{
    std::string desiredCommandTopicName, outputTopicName;

    n.param<std::string>(INPUT_TOPIC_PARAM, desiredCommandTopicName, DEFAULT_INPUT_TOPIC);
    n.param<std::string>(OUTPUT_TOPIC_PARAM, outputTopicName, DEFAULT_OUTPUT_TOPIC);

    publishTimer = n.createTimer(ros::Duration(1.0/COMMAND_RATE), &CommandRepeater::publishTimerCallback, this);
    desiredCommandTopic = n.subscribe(desiredCommandTopicName, 10000, &CommandRepeater::updateDesiredCommand, this);
    outputCommandTopic = n.advertise<geometry_msgs::Twist>(outputTopicName, 100);

    lastCommandReceiveTime = ros::Time(0);
}

void CommandRepeater::spin()
{
    ros::spin();
}

geometry_msgs::Twist CommandRepeater::idleTwistCommand()
{
    geometry_msgs::Twist retVal;
    retVal.angular.x = 0.0;
    retVal.angular.y = 0.0;
    retVal.angular.z = 0.0;
    retVal.linear.x = 0.0;
    retVal.linear.y = 0.0;
    retVal.linear.z = 0.0;

    return retVal;
}

void CommandRepeater::updateDesiredCommand(const geometry_msgs::Twist::ConstPtr& msg)
{
    lastCommandReceiveTime = ros::Time::now();
    desiredCommand = *msg;
}

void CommandRepeater::publishTimerCallback(const ros::TimerEvent& msg)
{
    publishCommand(desiredCommand);
}

void CommandRepeater::publishCommand(const geometry_msgs::Twist command)
{
    if(ros::Time::now() - lastCommandReceiveTime > ros::Duration(DEFAULT_TIMEOUT))
    {
        outputCommandTopic.publish(idleTwistCommand());
    } else {
        outputCommandTopic.publish(command);
    }
}
