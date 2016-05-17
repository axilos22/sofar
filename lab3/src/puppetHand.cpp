/**
\file puppetHand.cpp
\brief puppet.cpp creates a puppet_hand node which moves the left hand on top of the righ hand
\author  AJ & BG
\date 18/05/2016
*/

//Cpp
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

//ROS
#include "ros/ros.h"

//ROS msgs
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>

//DEBUG
#define VERBOSE 0

//By default we puppet the right arm to the left
const std::string _defaultArm = "left",
    _nodeName = "puppet_hand",
    _jointStateTopic = "/robot/joint_states";
const unsigned int _freq = 10; //10Hz frequency

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, _nodeName);
    ROS_INFO("%s connected to roscore",_nodeName.c_str());
    ros::NodeHandle nh_("~");//ROS node Handler absolute & relative path
    ros::Rate rate(_freq);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("@%s Node terminated\n",_nodeName.c_str());
}
