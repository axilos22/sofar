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
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
sensor_msgs::JointState mySensorMsg;
int count=0;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  //FILL VECTOR
  mySensorMsg.name.push_back("sensor1");
	mySensorMsg.position.push_back(33.33);
	mySensorMsg.velocity.push_back(11.11);
	mySensorMsg.effort.push_back(22.22);	
	
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher jointPub = n.advertise<sensor_msgs::JointState>("sensorPublisher",5);
  ros::Rate loop_rate(.5);
  
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    jointPub.publish(mySensorMsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
