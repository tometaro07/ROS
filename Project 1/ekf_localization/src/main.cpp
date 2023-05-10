#include "kalman.hpp"

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ekf_localization_node");

    if(argc != 2)
    {
        std::cout << "Usage: "<< argv[0] << " MAP_FILE.pgm" << std::endl;
        exit(1);
    }

    EKFnode k;
    k.spin();

    return 0;
}
