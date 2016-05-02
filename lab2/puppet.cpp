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
#define CONTROLLING_RIGHT 0
#define CONTROLLING_LEFT 1
#define DEFAULT_Q_SIZE 10

int _freq = 1; //1Hz frequency
std::vector<std::string> _jointNameList ;
sensor_msgs::JointState _joints,_jointsP;
ros::Publisher jointPub;
unsigned int _armControlled = CONTROLLING_LEFT;

//function for scope
void jointStateCallback(sensor_msgs::JointState joint_state);
void performProcessing();


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
    //We can launch the processing to transmit to the other arm.
    performProcessing();
}
/**
 * @brief performProcessing perform the change of sign to the new joint state
 * perform the conversion from joint state message to joint command message
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

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "puppet_node");
    ROS_INFO("mirror process connected to roscore");
    ros::NodeHandle nh_("~");//ROS Handler all namespaces
    _jointNameList.push_back("left_s0");
    //0 means we control right, 1 left
//    nh_.param("armControlled",_armControlled,CONTROLLING_LEFT);
//    ROS_INFO("The arm controlled is %d",_armControlled);

    //declaring subscriber
    ros::Subscriber jointsSubsciber = nh_.subscribe("/robot/joint_states",DEFAULT_Q_SIZE,jointStateCallback);

    //declaring publisher
    jointPub = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limbs/left/joint_command",5);
    ros::Rate rate(_freq);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
