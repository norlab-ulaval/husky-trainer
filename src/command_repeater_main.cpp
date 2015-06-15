
#include "husky_trainer/CommandRepeater.h"

#define NODE_NAME "command_repeater"

int main(int argc, char**argv)
{
    ros::init(argc,argv, NODE_NAME);
    ros::NodeHandle n("~");

    CommandRepeater repeater(n);
    repeater.spin();

    return 0;
}
