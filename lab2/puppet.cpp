/**
\file
\brief
\author(s)
\date       08/04/2016
*/

//Cpp
#include <iostream>
#include <sstream.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

//ROS
#include "ros/ros.h"

//ROS msgs
#include "std_msg/Int16.h"
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>

#define DEFAULT_ARM_RIGHT true

/*
 *   You may have a number of globals declared here...
 */
int _freq = 100;
bool _isRightArm;
std::string _jointName;
sensor_msgs::JointState _joints;

/**
  \fn <return type> <function name>(<argument list>)
  \brief At least a one line description of any function.
  */
void getSensorPos(sensor_msgs::JointState joint_state) {
    _joints.clear();
    _joints = joint_state;
    for(int i =0;i<joint_state.size();i++) {
        _joints.name.push_back(joint_state.name[i]);
        _joints.position.push_back(joint_state.position[i]);
        _joints.position.push_back(joint_state.velocity[i]);
    }
    //Here my _joint variable is up to date
}
/**
 * @brief performProcessing perform the change of sign to the new
 */
void performProcessing() {

}

void setSlavePos() {

}


int main (int argc, char** argv)
{
  //ROS Initialization
  ros::init(argc, argv, "mirror");
  ROS_INFO("mirror connected to roscore");
  ros::NodeHandle nh_("~");//ROS Handler - local namespace.

  nh_.param("arm",_isRightArm,DEFAULT_ARM_RIGHT);
/*
 *   Now retrieve parameter values if any...
 */

  //Subscribing
  
/*
 *   Declare here all your subscriptions.
 */

  //Publishing  

/*
 *   Declare here all your publications.
 */

/*
 *   Perform here any initialization code you may need.
 */

/*
 *   Now the main lopp
 */

    ros::Rate rate(_freq);
    while (ros::ok()) {

         /*
          *   Your code goes here
          */

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
