//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <ostream>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"

//ROS msgs
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void sensorCallback(const sensor_msgs::JointState &msg) {
	ROS_INFO("I just got a joint state !");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("sensorPublisher",10,sensorCallback);
  ros::spin();

  return 0;
}
