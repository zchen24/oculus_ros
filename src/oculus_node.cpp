#include <iostream>
#include <ros/ros.h>
#include "oculus_rift.h"


int main(int argc, char** argv)
{
    std::cout << "oculus node" << std::endl;

    ros::init(argc, argv, "oculus_node");
    ros::NodeHandle node;
    ros::NodeHandle local_node("~");
    double frequency(10.0);
    local_node.getParam("frequency", frequency);

    oculus_ros::OculusRift oculus(node);
    oculus.Init();

//    oculus_driver::OculusRos oculus(node);
    ros::Rate rate(frequency);
    while(ros::ok())
    {
        oculus.UpdateSensors();
        ros::spinOnce();
        rate.sleep();
    }
}
