
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
#include "husky_trainer/controllerConfig.h"

class Repeat {
public:
    Repeat(ros::NodeHandle n);
    ~Repeat();
    void spin();

private:
    enum Status { PLAY = 0, PAUSE, ERROR };
    typedef boost::tuple<double, double, double> IcpError; // Ordered as follows: x, y, theta.
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    // Parameter names.
    static const std::string LAMBDA_X_PARAM;
    static const std::string LAMBDA_Y_PARAM;
    static const std::string LAMBDA_THETA_PARAM;
    static const std::string LOOKAHEAD_PARAM;
    static const std::string SOURCE_TOPIC_PARAM;
    static const std::string COMMAND_OUTPUT_PARAM;
    static const std::string WORKING_DIRECTORY_PARAM;

    // Default values.
    static const double DEFAULT_LAMBDA_X;
    static const double DEFAULT_LAMBDA_Y;
    static const double DEFAULT_LAMBDA_THETA;
    static const double DEFAULT_LOOKAHEAD;
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
    IcpError currentError;
    double lambdaX, lambdaY, lambdaTheta, lookahead;
    double lpFilterTimeConst; // Low-pass filter time constant.
    std::string sourceTopicName;
    tf::StampedTransform tFromLidarToRobot;
    ros::Time baseSimTime;
    ros::Time timePlaybackStarted;
    ros::Rate loopRate;

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
    dynamic_reconfigure::Server<husky_trainer::controllerConfig> drServer;

    // Functions.
    static void loadAnchorPoints(std::string filename, std::vector<AnchorPoint>& out);
    static void loadCommands(std::string filename, std::vector<geometry_msgs::TwistStamped>& out);
    static void loadPositions(std::string filename, std::vector<geometry_msgs::PoseStamped>& out);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg);
    void joystickCallback(sensor_msgs::Joy::ConstPtr msg);
    void controllerParametersCallback(husky_trainer::controllerConfig &params, uint32_t level);

    // Time management.
    void switchToStatus(Status desiredStatus);
    void pausePlayback();
    void startPlayback();
    ros::Time simTime();

    void updateError(const sensor_msgs::PointCloud2& msg);
    geometry_msgs::Twist commandOfTime(ros::Time time);
    geometry_msgs::Pose poseOfTime(ros::Time time);
    geometry_msgs::Twist errorAdjustedCommand(geometry_msgs::Twist command, IcpError error);
    static void publishError(IcpError error, ros::Publisher topic);
};

#endif
