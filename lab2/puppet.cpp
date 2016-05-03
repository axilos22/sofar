/**
\file puppet.cpp
\brief puppet.cpp creates a puppet nodes which mimic the movement of one arm to another
\author  AJ & BG
\date 03/05/2016
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

//DEBUG
#define VERBOSE 0
#define DEFAULT_Q_SIZE 10 //default queue size
#define CONTROL_POSITION 1 //default control mode is 1
#define CONTROL_VELOCITY 0
//By default we puppet the right arm to the left
const std::string _defaultArm = "left",
    _nodeName = "puppet_node",
    _jointStateTopic = "/robot/joint_states";
std::string _masterArm,_slaveArm;
const unsigned int _freq = 10; //10Hz frequency
int _controlMode;
std::vector<std::string> _jointNameList ;
sensor_msgs::JointState _joints,_jointsP;
ros::Publisher jointPub;

//function for scope
void jointStateCallback(sensor_msgs::JointState joint_state);
void performProcessing();

/**
 * @brief jointStateCallback is executed when the state of joints are published.
 * @param joint_state the joint state message of the robot.
 */
void jointStateCallback(sensor_msgs::JointState joint_state) {
    //ROS_INFO("jointStateCallback");
    //cleaning last joint state
    _joints.name.clear();
    _joints.position.clear();
    _joints.velocity.clear();
    //store all the joint state published into local variable
    for(int i =0;i<joint_state.name.size();i++) {
        _joints.name.push_back(joint_state.name[i]);
        _joints.position.push_back(joint_state.position[i]);
        _joints.velocity.push_back(joint_state.velocity[i]);
    }
    performProcessing();
}
/**
 * @brief performProcessing 1)perform the change of sign to the new joint state (TODO)
 *                          2)perform the conversion from joint state message to joint command message
 */
void performProcessing() {
    baxter_core_msgs::JointCommand msg_command;
//    std::string _masterArm = "right";
    _jointsP.name.clear();
    _jointsP.position.clear();
    _jointsP.velocity.clear();
    for(int i =0;i<_joints.name.size();i++) {
        std::size_t found = _joints.name[i].find(_masterArm);
        if (found!=std::string::npos) {
            ROS_INFO("i found %s in %s",_masterArm.c_str(),_joints.name[i].c_str());
            _jointsP.name.push_back(_joints.name[i]);
            _jointsP.position.push_back(_joints.position[i]);
            _jointsP.velocity.push_back(_joints.velocity[i]);
        }
    }
//    std::string publishSide = "left";
    for(int h=0; h<_jointsP.name.size();h++) {
        ROS_INFO("before %s",_jointsP.name[h].c_str());
        _jointsP.name[h].replace(0,5,_slaveArm);
        ROS_INFO("after %s",_jointsP.name[h].c_str());
    }
    if(_controlMode==CONTROL_POSITION) {
        msg_command.mode = msg_command.POSITION_MODE;
    } else {
        msg_command.mode = msg_command.VELOCITY_MODE;
    }

    for(int j=0;j<_jointsP.name.size();j++) {
        msg_command.names.push_back(_jointsP.name[j]);
        if(_controlMode==CONTROL_POSITION)
            msg_command.command.push_back(_jointsP.position[j]);
        else
            msg_command.command.push_back(_jointsP.velocity[j]);
    }
    jointPub.publish(msg_command);
}
/**
 * @brief fillJointName fill the joint name list with all the joints.
 */
void fillJointName(std::string armSide) {
    _jointNameList.clear();
    _jointNameList.push_back(armSide+"_e0");
    _jointNameList.push_back(armSide+"_e1");
    _jointNameList.push_back(armSide+"_s0");
    _jointNameList.push_back(armSide+"_s1");
    _jointNameList.push_back(armSide+"_w0");
    _jointNameList.push_back(armSide+"_w1");
    _jointNameList.push_back(armSide+"_w2");
}
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
    ROS_INFO("@%s connected to roscore",_nodeName.c_str());
    ros::NodeHandle nh_("~");//ROS Handler absolute & relative paths
    //Side of the arm controlled
    std::string armControlled;
    nh_.param<std::string>("armControlled",armControlled,_defaultArm);
    //Controlling in position or velocity
    nh_.param<int>("controlMode",_controlMode,CONTROL_POSITION);

    //User input verification
    if(armControlled.compare("left")==0||armControlled.compare("right")==0) {
        ROS_ERROR("@%s Invalid arm side provided: %s",_nodeName.c_str(),armControlled.c_str());
        return -1;
    } else {
        ROS_INFO("The arm controlled is %s",armControlled.c_str());
        if(armControlled.compare("left")==0) {
            _masterArm="right";
            _slaveArm="left";
        } else {
            _masterArm="left";
            _slaveArm="right";
        }
    }
    if(_controlMode!=CONTROL_POSITION||_controlMode!=CONTROL_VELOCITY) {
        ROS_ERROR("@%s Invalid control mode provided: %d",_nodeName.c_str(),_controlMode);
        return -1;
    } else {
        if (_controlMode==CONTROL_POSITION)
            ROS_INFO("Robot controlled in position");
        if (_controlMode==CONTROL_VELOCITY)
            ROS_INFO("Robot controlled in velocity");
    }
    fillJointName(_masterArm);

    //declaring subscriber
    ros::Subscriber jointsSubsciber = nh_.subscribe(_jointStateTopic,DEFAULT_Q_SIZE,jointStateCallback);

    //declaring publisher
    std::string jointCommandTopic = "/robot/limbs/"+_slaveArm+"/joint_command";
    jointPub = nh_.advertise<baxter_core_msgs::JointCommand>(jointCommandTopic,5);
    ros::Rate rate(_freq);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("@%s Node terminated\n",_nodeName.c_str());
}
