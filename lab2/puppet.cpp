/**
\file puppet.cpp
\brief puppet.cpp creates a puppet nodes which mimic the movement of one arm to another
\author  AJ & BG
\date 08/04/2016
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
#include <std_msg/Int16.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>

//By default we puppet the right arm to the left
#define DEFAULT_ARM_RIGHT true
#define DEFAULT_ARM_LEFT false
#define DEFAULT_Q_SIZE 10

int _freq = 100;
bool _isRightArmPuppet, _isLeftArmPuppet;
std::vector<std::string> _jointNameList = {"left_s0"};
sensor_msgs::JointState _joints;

/**
  \fn void getSensorPos(joint_state)
  \brief This callback stores into the local variable, the updated joint list.
  */
void jointStateCallback(sensor_msgs::JointState joint_state) {
    _joints.clear();
    //store all the joint state published into local variable
    for(int i =0;i<joint_state.size();i++) {
        _joints.name.push_back(joint_state.name[i]);
        _joints.position.push_back(joint_state.position[i]);
        _joints.position.push_back(joint_state.velocity[i]);
    }
    //Here my _joint variable is up to date
}
/**
 * @brief performProcessing perform the change of sign to the new joint state
 */
void performProcessing() {

}
/**
 * @brief publish the new joint position to the puppeted arm
 */ 
void setSlavePos() {

}


int main (int argc, char** argv)
{
  //ROS Initialization
  ros::init(argc, argv, "mirror");
  ROS_INFO("mirror connected to roscore");
  ros::NodeHandle nh_("~");//ROS Handler - local namespace.

  nh_.param("righArmPuppet",_isRightArmPuppet,DEFAULT_ARM_RIGHT);
  nh_.param("leftArmPuppet",_isLeftArmPuppet,DEFAULT_ARM_LEFT);
  
  if(_isRightArmPuppet) {
	  ROS_INFO("Puppet arm is the right");
  } else {
	  ROS_INFO("Puppet arm is the left");
  }
  //Subscribing to the master arm joint state
  ros::Subscriber jointsSubsciber = nh_.subscribe("jointState",DEFAULT_Q_SIZE,jointStateCallback);
  
  //Publishing into the slave arm joint state
/*
 *   Perform here any initialization code you may need.
 */

/*
 *   Now the main lopp
 */
	ros::Rate rate(_freq);
    while (ros::ok()) {         
		//check for callback execution
		//...
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
