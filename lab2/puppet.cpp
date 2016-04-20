/**
\file puppet.cpp
\brief puppet.cpp creates a puppet nodes which mimic the movement of one arm to another
\author  AJ & BG
\date 20/04/2016
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

//By default we puppet the right arm to the left
#define DEFAULT_ARM_RIGHT true
#define DEFAULT_ARM_LEFT false
#define DEFAULT_Q_SIZE 10

int _freq = 1; //1Hz frequency
bool _isRightArmPuppet, _isLeftArmPuppet;
std::vector<std::string> _jointNameList ;

sensor_msgs::JointState _joints,_jointsP;
ros::Publisher jointPub;

/**
  \fn void getSensorPos(joint_state)
  \brief This callback stores into the local variable, the updated joint list.
  */
void jointStateCallback(sensor_msgs::JointState joint_state) {
    //ROS_INFO("jointStateCallback");
    _joints.name.clear();
    _joints.position.clear();
    _joints.velocity.clear();
    //store all the joint state published into local variable
    for(int i =0;i<joint_state.name.size();i++) {
        _joints.name.push_back(joint_state.name[i]);
        _joints.position.push_back(joint_state.position[i]);
        _joints.velocity.push_back(joint_state.velocity[i]);
    }
    //Here my _joint variable is up to date
}
/**
 * @brief performProcessing perform the change of sign to the new joint state
 */
void performProcessing() {
    baxter_core_msgs::JointCommand msg_command;
    std::string mySide = "right";
    _jointsP.name.clear();
    _jointsP.position.clear();
    _jointsP.velocity.clear();
    for(int i =0;i<_joints.name.size();i++) {
        std::size_t found = _joints.name[i].find(mySide);
         if (found!=std::string::npos) {
             ROS_INFO("i found %s in %s",mySide.c_str(),_joints.name[i].c_str());
              _jointsP.name.push_back(_joints.name[i]);
              _jointsP.position.push_back(_joints.position[i]);
              _jointsP.velocity.push_back(_joints.velocity[i]);
         }
    }
    std::string publishSide = "left";
    for(int h=0; h<_jointsP.name.size();h++) {
        ROS_INFO("before %s",_jointsP.name[h].c_str());
        _jointsP.name[h].replace(0,5,publishSide);
        ROS_INFO("after %s",_jointsP.name[h].c_str());
    }
    msg_command.mode = msg_command.POSITION_MODE;
    for(int j=0;j<_jointsP.name.size();j++) {
        msg_command.names.push_back(_jointsP.name[j]);
        msg_command.command.push_back( _jointsP.position[j]);
    }
    jointPub.publish(msg_command);
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
  ROS_INFO("mirror process connected to roscore");
  ros::NodeHandle nh_("~");//ROS Handler - local namespace.


  _jointNameList.push_back("left_s0");
  nh_.param("righArmPuppet",_isRightArmPuppet,DEFAULT_ARM_RIGHT);
  nh_.param("leftArmPuppet",_isLeftArmPuppet,DEFAULT_ARM_LEFT);

  if(_isRightArmPuppet) {
      ROS_INFO("Puppet arm is the right");
  } else {
      ROS_INFO("Puppet arm is the left");
  }
  //declaring subscriber
  ros::Subscriber jointsSubsciber = nh_.subscribe("/robot/joint_states",DEFAULT_Q_SIZE,jointStateCallback);

  //declaring publisher
  jointPub = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limbs/left/joint_command",5);
    ros::Rate rate(_freq);
    while (ros::ok()) {
        //Subscribing to the master arm joint state -- implicit
        //Publishing into the slave arm joint state
        //Perform processing to invert some joints
        performProcessing();
        //jointPub.publish(_jointsP);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
