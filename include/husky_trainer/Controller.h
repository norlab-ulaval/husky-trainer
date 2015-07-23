
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <boost/tuple/tuple.hpp>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include "husky_trainer/RepeatConfig.h"
#include "husky_trainer/TrajectoryError.h"


class Controller {
    public:
        Controller(ros::NodeHandle n);
        geometry_msgs::Twist correctCommand(geometry_msgs::Twist command);
        void updateError(husky_trainer::TrajectoryError newError);
        void updateParams(husky_trainer::RepeatConfig& params);

    private:
        static const double SAMPLING_PERIOD;
        static const std::string CORRECTED_ERROR_TOPIC;

        husky_trainer::TrajectoryError currentError;
        double lambdaX, lambdaY, lambdaTheta;
        double lpFilterTimeConstant;
        double minLinearSpeed, maxLinearSpeed, minAngularSpeed, maxAngularSpeed;

        ros::Publisher correctedErrorTopic;

        geometry_msgs::Twist cutoff(geometry_msgs::Twist command);
        double iir(double old, double input);
        geometry_msgs::Twist proportionalGain(geometry_msgs::Twist input);
};

#endif
