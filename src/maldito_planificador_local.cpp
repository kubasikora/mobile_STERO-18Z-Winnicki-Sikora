#include "ros/ros.h"
#include <iostream>
#include <vector>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stero/planificador");
    ros::NodeHandle nodeHandle;

    ros::Rate rate(1);
    int n = 0;

    while(ros::ok())
    {
        std::cout<<"Maldito planificador local #"<<n<<": aby zakonczyc: Ctrl + c\n\r";

        rate.sleep();
        n++;
    }
}