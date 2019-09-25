/******************************************************************************
    * labjack_node.cpp -- LabJack T7 ROS node                                 *
    *                                                                         *
    * Carnegie Mellon Univesity                                               *
    *                                                                         *
    * Author:   Ford, Jordan                                                  *
    *           Papincak, Lawrence                                            *
    *                                                                         *
    * Purpose:  Configures and publishes data read from LabJack T7 ADC.       *                                                       *
    *                                                                         *
    * Usage:                                                                  *
    **************************************************************************/

#include "ros/ros.h"
#include "robot_config/Body_Temperatures.h"
#include "robot_config/Forward_Range.h"

#include "LabJackM.h"
#include "labjack.h"
#include "labjack_pinout.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <std_msgs/Float32.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "labjack_node");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  // Get parameters
  std::string ip;
  std::string temperature_sensors_topic;
  std::string dt50_sensors_topic;
  std::string topic_name_logger;

  float update_rate;
  int handle;
  double value_logging = 0;
  int start_stop = 0;

  n_private.param<std::string>("IP", ip, "10.0.10.15");
  n_private.param<std::string>("body_temperature_topic", temperature_sensors_topic, "/body_temperature");
  n_private.param<std::string>("dt50_sensor_topic", dt50_sensors_topic, "/dt50_forward_range");
  n_private.param<std::string>("topic_name_logger" , topic_name_logger , "/data_logging");

  n_private.param<float>("update_rate", update_rate, 16.666);

  // Initialize ADC and configure ethernet
  ROS_INFO("Labjack IP: %s",ip.c_str());
  labjack_adc adc(ip, "0");
  handle = adc.tcpConfig();

  /* Setup publisher */
  ros::Publisher temperature_publisher = n.advertise<robot_config::Body_Temperatures>(temperature_sensors_topic, 10);
  ros::Publisher forward_range_publisher = n.advertise<robot_config::Forward_Range>(dt50_sensors_topic, 10);

  // The SICK IMA30 is advertised to produce valid data after no more than 60ms.
  ros::Rate loop_rate(update_rate);
  std_msgs::Float32 msg_log;

  robot_config::Body_Temperatures temp_msg;
  robot_config::Forward_Range range_msg;

  while (ros::ok())
  {
    temp_msg.header.stamp = ros::Time::now();
    range_msg.header.stamp = ros::Time::now();

    temp_msg.temperature0 = adc.readAnalog(BODY_TEMP0);
    temp_msg.temperature1 = adc.readAnalog(BODY_TEMP1);

    range_msg.left_voltage = adc.readAnalog(DT50_LEFT);
    range_msg.left_distance = range_msg.left_voltage*range_msg.volt2meters_m + range_msg.volt2meters_b;

    range_msg.right_voltage = adc.readAnalog(DT50_RIGHT);
    range_msg.right_distance = range_msg.right_voltage*range_msg.volt2meters_m + range_msg.volt2meters_b;

    bool temp_read_fail = (temp_msg.temperature0 == -1) || (temp_msg.temperature1 == -1);
    bool range_read_fail = (range_msg.left_voltage == -1) || (range_msg.right_voltage == -1);

    if (!temp_read_fail)
    {
      temperature_publisher.publish(temp_msg);
    }
    else
    {
      std::cerr << "WARN: Failed to read ADC for Body Temperatures" << std::endl;
    }

    if (!range_read_fail)
    {
      forward_range_publisher.publish(range_msg);
    }
    else
    {
     std::cerr << "WARN: Failed to read ADC for Forward_Range" << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
