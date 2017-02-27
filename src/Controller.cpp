
#include "husky_trainer/Controller.h"

const double Controller::SAMPLING_PERIOD = 0.33;
const std::string Controller::CORRECTED_ERROR_TOPIC = 
    "/teach_repeat/corrected_error";

Controller::Controller(ros::NodeHandle n) 
{
    husky_trainer::TrajectoryError error;
    error.x = 0.0;
    error.y = 0.0;
    error.theta = 0.0;
    currentError = error;

    correctedErrorTopic = 
        n.advertise<husky_trainer::TrajectoryError>(CORRECTED_ERROR_TOPIC, 100);
}
        
geometry_msgs::Twist Controller::correctCommand(geometry_msgs::Twist command)
{
    return cutoff(proportionalGain(command));
}

void Controller::updateError(husky_trainer::TrajectoryError newError)
{
    double filteredX = iir(currentError.x, newError.x);
    currentError.x = filteredX;
    currentError.y = newError.y;
    currentError.theta = newError.theta;

    correctedErrorTopic.publish(currentError);
}

geometry_msgs::Twist Controller::cutoff(geometry_msgs::Twist command)
{
    double newLinear = 
        std::min(maxLinearSpeed, std::max(minLinearSpeed, command.linear.x));

    double newAngular = 
        std::min(maxAngularSpeed, std::max(minAngularSpeed, command.angular.z));

    geometry_msgs::Twist returnValue;
    returnValue.angular.z = newAngular;
    returnValue.linear.x = newLinear;
    return returnValue;
}

double Controller::iir(double old, double input)
{
    double gain = exp(-SAMPLING_PERIOD / lpFilterTimeConstant);
    return old + (1.0 - gain) * (input - old);
}

geometry_msgs::Twist Controller::proportionalGain(geometry_msgs::Twist input)
{
    float newAngular = 
        input.angular.z 
        + lambdaY * currentError.y
        - lambdaTheta * currentError.theta;
    float newLinear = 
        input.linear.x * cos(currentError.y) 
        - lambdaX * currentError.x;

    input.angular.z = newAngular;
    input.linear.x = newLinear;

    return input;
}

void Controller::updateParams(husky_trainer::RepeatConfig &params)
{
    lambdaX = params.lambda_x;
    lambdaY = params.lambda_y; 
    lambdaTheta = params.lambda_t;
    lpFilterTimeConstant = params.lp_filter_time_const;

    minLinearSpeed = params.min_linear_speed;
    maxLinearSpeed = params.max_linear_speed;
    minAngularSpeed = params.min_angular_speed;
    maxAngularSpeed = params.max_angular_speed;
}

