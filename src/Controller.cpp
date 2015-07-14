
#include "husky_trainer/ControllerConfig.h"
#include "husky_trainer/Controller.h"

const double Controller::SAMPLING_PERIOD = 0.33;

Controller::Controller() : currentError(IcpError(0.0,0.0,0.0))
{
    // Setup the dynamic reconfiguration server.
    dynamic_reconfigure::Server<husky_trainer::ControllerConfig>::CallbackType callback;
    callback = boost::bind(&Controller::paramCallback, this, _1, _2);
    drServer.setCallback(callback);
}
        
geometry_msgs::Twist Controller::correctCommand(geometry_msgs::Twist command)
{
    return cutoff(proportionalGain(command));
}

void Controller::updateError(IcpError newError)
{
    double filteredX = iir(currentError.get<0>(), newError.get<0>());
    currentError = IcpError(filteredX, currentError.get<1>(), currentError.get<2>());
}

geometry_msgs::Twist Controller::cutoff(geometry_msgs::Twist command)
{
    double newLinear = 
        command.linear.x > maxLinearSpeed ? 
            maxLinearSpeed : 
            command.linear.x < minLinearSpeed ? 
                minLinearSpeed : 
                command.linear.x;

    double newAngular = 
        command.angular.z > maxAngularSpeed ? 
            maxAngularSpeed :
            command.angular.z < minAngularSpeed ?
                minAngularSpeed:
                command.angular.z;

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
        + lambdaY * currentError.get<1>() 
        - lambdaTheta * currentError.get<2>();
    float newLinear = 
        input.linear.x * cos(currentError.get<1>()) 
        - lambdaX * currentError.get<0>();

    input.angular.z = newAngular;
    input.linear.x = newLinear;

    return input;
}

void Controller::paramCallback(husky_trainer::ControllerConfig &params, uint32_t level)
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

