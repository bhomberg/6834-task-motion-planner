#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "test_code/SerialMsg.h"
#include <string>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include "baxter_soft_hand/flex_value.h"
#include "baxter_soft_hand/finger_target.h"

serial::Serial my_serial("/dev/ttyACM0", 9600, serial::Timeout::simpleTimeout(1000));

void setPosition(const baxter_soft_hand::finger_target::ConstPtr& msg)
{
  int finger = msg->finger;
  unsigned short target = msg->target; 

  if(target == 4000)
    {
      ROS_INFO("Invalid setpoint: [%d]", target);
      
      const char command[] = {0xB3};
      size_t bytes_wrote = my_serial.write(command);
    }
  else if(target == 5000)
    {
      ROS_INFO("Reading current");
      
      if(finger == 1){
	const char command[] = {0xAA, 0x09, 0x8F};
	size_t bytes_wrote = my_serial.write(command);
      }else if(finger == 2){
	const char command[] = {0xAA, 0x0A, 0x8F};
	size_t bytes_wrote = my_serial.write(command);
      }else if(finger == 3){
	const char command[] = {0xAA, 0x0B, 0x8F};
	size_t bytes_wrote = my_serial.write(command);
      }else if(finger == 4){
	const char command[] = {0xAA, 0x08, 0x8F};
	size_t bytes_wrote = my_serial.write(command);
      }else{ROS_INFO("Invalid finger number.");}

      std::string result = my_serial.read(1);
      //const char* result2 = result.c_str();
      //result = "0x0A";
      unsigned int x;
      std::stringstream ss;
      ss << std::dec << result;
      ss >> x;

      //int num = std::stoul(result2, NULL, 16);
      const char* result2 = result.c_str();
      ROS_INFO("Current: %s", result2);
      ROS_INFO("Current: %d", x);
    }
  else if(target < 3500 && target > 0)
    {
      ROS_INFO("Valid setpoint: [%d]", target);
      
      if(finger == 1){
	const char command[] = {0xAA, 0x09, 0x40 + (target & 0x1F), (target >> 5) & 0x7F};
	size_t bytes_wrote = my_serial.write(command);
      }else if(finger == 2){
	const char command[] = {0xAA, 0x0A, 0x40 + (target & 0x1F), (target >> 5) & 0x7F};
	size_t bytes_wrote = my_serial.write(command);
      }else if(finger == 3){
	const char command[] = {0xAA, 0x0B, 0x40 + (target & 0x1F), (target >> 5) & 0x7F};
	size_t bytes_wrote = my_serial.write(command);
      }else if(finger == 4){
	const char command[] = {0xAA, 0x08, 0x40 + (target & 0x1F), (target >> 5) & 0x7F};
	size_t bytes_wrote = my_serial.write(command);
      }else{ROS_INFO("Invalid finger number.");}
    }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Motor_Controller_Node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("finger_target", 1000, setPosition);
 
  ROS_INFO("Is the serial port open?");
  if(my_serial.isOpen())
    ROS_INFO("Yes!");
  else
    ROS_INFO("No :(");

  const char command[] = {0xAA};
  size_t bytes_wrote = my_serial.write(command);

  ros::spin();

  return 0;
}
