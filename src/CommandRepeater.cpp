
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
    desiredCommandTopic = n.subscribe(desiredCommandTopicName, 100, &CommandRepeater::updateDesiredCommand, this);
    outputCommandTopic = n.advertise<geometry_msgs::Twist>(outputTopicName, 100);
}

void CommandRepeater::spin()
{
    ros::spin();
}

void CommandRepeater::updateDesiredCommand(const geometry_msgs::Twist::ConstPtr& msg)
{
    desiredCommand = *msg;
}

void CommandRepeater::publishTimerCallback(const ros::TimerEvent& msg)
{
    publishCommand(desiredCommand);
}

void CommandRepeater::publishCommand(const geometry_msgs::Twist command)
{
    outputCommandTopic.publish(command);
}
