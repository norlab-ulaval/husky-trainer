
#include <ros/ros.h>
#include "husky_trainer/CloudRecorder.h"

#define NODE_NAME "cloud_recorder"

int main(int argc, char**argv)
{
    ros::init(argc,argv, NODE_NAME);
    ros::NodeHandle n("~");

    CloudRecorder recorder(n);
    recorder.spin();

    return 0;
}
