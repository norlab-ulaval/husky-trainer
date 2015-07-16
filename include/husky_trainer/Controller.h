
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <boost/tuple/tuple.hpp>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include "husky_trainer/ControllerConfig.h"
#include "husky_trainer/TrajectoryError.h"


class Controller {
    public:
        // IcpError is ordered as follows: x, y, theta.
        typedef boost::tuple<double, double, double> IcpError; 

        Controller(ros::NodeHandle n);
        geometry_msgs::Twist correctCommand(geometry_msgs::Twist command);
        void updateError(IcpError newError);

    private:
        static const double SAMPLING_PERIOD;
        static const std::string CORRECTED_ERROR_TOPIC;

        IcpError currentError;
        double lambdaX, lambdaY, lambdaTheta;
        double lpFilterTimeConstant;
        double minLinearSpeed, maxLinearSpeed, minAngularSpeed, maxAngularSpeed;
        dynamic_reconfigure::Server<husky_trainer::ControllerConfig> drServer;

        ros::Publisher correctedErrorTopic;

        void paramCallback(husky_trainer::ControllerConfig &params, uint32_t level);

        geometry_msgs::Twist cutoff(geometry_msgs::Twist command);
        double iir(double old, double input);
        geometry_msgs::Twist proportionalGain(geometry_msgs::Twist input);
};

#endif
