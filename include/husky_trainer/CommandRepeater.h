
#ifndef COMMAND_REPEATER_H
#define COMMAND_REPEATER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define COMMAND_RATE 50
#define INPUT_TOPIC_PARAM "input"
#define OUTPUT_TOPIC_PARAM "output"
#define DEFAULT_INPUT_TOPIC "/desired_command"
#define DEFAULT_OUTPUT_TOPIC "/husky/cmd_vel"

class CommandRepeater {
public:
    CommandRepeater(ros::NodeHandle n);
    void spin();

private:
    geometry_msgs::Twist desiredCommand;
    ros::Subscriber desiredCommandTopic;
    ros::Publisher outputCommandTopic;
    ros::Timer publishTimer;

    void publishTimerCallback(const ros::TimerEvent&);
    void updateDesiredCommand(const geometry_msgs::Twist::ConstPtr& msg);
    void publishCommand(const geometry_msgs::Twist msg);
};

#endif
