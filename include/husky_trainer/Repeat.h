
#ifndef REPEAT_CLASS_H
#define REPEAT_CLASS_H

#include <string>
#include <vector>
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <dynamic_reconfigure/server.h>

#include "pointmatcher_ros/MatchClouds.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/point_cloud.h"

#include "husky_trainer/CommandRepeater.h"
#include "husky_trainer/AnchorPoint.h"
#include "husky_trainer/PointMatching.h"
#include "husky_trainer/AnchorPointSwitch.h"
#include "husky_trainer/Controller.h"
#include "husky_trainer/TrajectoryError.h"
#include "husky_trainer/RepeatConfig.h"

class Repeat {
public:
    Repeat(ros::NodeHandle n);
    ~Repeat();
    void spin();

private:
    enum Status { FORWARD = 0, REWIND, PAUSE, ERROR };
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    // Parameter names.
    static const std::string SOURCE_TOPIC_PARAM;
    static const std::string COMMAND_OUTPUT_PARAM;
    static const std::string WORKING_DIRECTORY_PARAM;

    // Default values.
    static const std::string DEFAULT_SOURCE_TOPIC;
    static const std::string DEFAULT_COMMAND_OUTPUT_TOPIC;

    // Other constants.
    static const double LOOP_RATE;
    static const std::string JOY_TOPIC;
    static const std::string REFERENCE_POSE_TOPIC;
    static const std::string ERROR_REPORTING_TOPIC;
    static const std::string AP_SWITCH_TOPIC;
    static const std::string CLOUD_MATCHING_SERVICE;
    static const std::string LIDAR_FRAME;
    static const std::string ROBOT_FRAME;
    static const std::string WORLD_FRAME;

    // Variables.
    Status currentStatus;
    Controller controller;
    ros::Duration lookahead;
    std::string sourceTopicName;
    tf::StampedTransform tFromLidarToRobot;
    ros::Time baseSimTime;
    ros::Time timePlaybackStarted;
    ros::Rate loopRate;
    dynamic_reconfigure::Server<husky_trainer::RepeatConfig> drServer;

    std::vector<AnchorPoint> anchorPoints;
    std::vector<geometry_msgs::PoseStamped> positions;
    std::vector<geometry_msgs::TwistStamped> commands;
    std::vector<geometry_msgs::PoseStamped>::iterator positionCursor;
    std::vector<geometry_msgs::TwistStamped>::iterator commandCursor;
    std::vector<AnchorPoint>::iterator anchorPointCursor;

    ros::Subscriber readingTopic;
    ros::Subscriber joystickTopic;
    ros::Publisher commandRepeaterTopic;
    ros::Publisher errorReportingTopic;
    ros::Publisher referencePoseTopic;
    ros::Publisher anchorPointSwitchTopic;
    ros::ServiceClient icpService;
    boost::mutex serviceCallLock;

    // Functions.
    static void loadAnchorPoints(std::string filename, std::vector<AnchorPoint>& out);
    static void loadCommands(std::string filename, std::vector<geometry_msgs::TwistStamped>& out);
    static void loadPositions(std::string filename, std::vector<geometry_msgs::PoseStamped>& out);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg);
    void joystickCallback(sensor_msgs::Joy::ConstPtr msg);

    // Time management.
    void switchToStatus(Status desiredStatus);
    void pausePlayback();
    void startPlayback();
    ros::Time simTime();
    ros::Time trySubtract(ros::Duration value, ros::Time from);

    void updateError(const sensor_msgs::PointCloud2& msg);
    void updateAnchorPoint();
    geometry_msgs::Twist commandOfTime(ros::Time time);
    static geometry_msgs::Twist reverseCommand(geometry_msgs::Twist input);
    geometry_msgs::Pose poseOfTime(ros::Time time);
    

    // Dynamic reconfigure.
    void paramCallback(husky_trainer::RepeatConfig &params, uint32_t level);
};

#endif
