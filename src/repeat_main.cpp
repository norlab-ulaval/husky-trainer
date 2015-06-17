
#include <ros/ros.h>
#include "husky_trainer/Repeat.h"

#define NODE_NAME "husky_repeat"

int main(int argc, char**argv)
{
    ros::init(argc,argv, NODE_NAME);
    ros::NodeHandle n("~");

    Repeat repeat(n);
    repeat.spin();

    return 0;
}
