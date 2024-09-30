#include "ros/ros.h"
#include <string>
#include "std_msgs/Duration.h"
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "music");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    double bpm = std::stof(argv[1]);

    int i = 0;
    ros::Time startTime;
    ros::Duration beat = ros::Duration(60.00 / bpm);
    ROS_INFO_STREAM(beat);
    sleep(2);
    while(ros::ok())
    {
        startTime = ros::Time::now();
        
        while(ros::Time::now() < startTime + beat);

        if(i % 2 == 0)
        {
            std::cout << " \\ ^_^ \\ \n" << std::endl;
        }
        else
        {
            std::cout << " / ^_^ / \n" << std::endl;
        }
        ++i;
        ros::spinOnce();        
    }
    return 0;
}