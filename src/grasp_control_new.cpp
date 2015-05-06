#include "ros/ros.h"
#include <string>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <unistd.h>
#include "baxter_soft_hand/flex_value.h"
#include "baxter_soft_hand/finger_target.h"
#include "baxter_soft_hand/flex_triple.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

int diam_estimates[3][10];
int counter = 0;
ros::Publisher target_pub;
ros::Publisher baxter_pub;
ros::Publisher flex_triple_pub;

void getFlexVal(const baxter_soft_hand::flex_value::ConstPtr &msg)
{
  //ROS_INFO("got flex value!");
  //ROS_INFO("finger num: %d", msg->finger_num);
  //ROS_INFO("value: %d", msg->value);

  counter = counter % 10;
  diam_estimates[msg->finger_num - 1][counter++] = msg-> value;
}

double sensorValToDiameter(int flex_val)
{
  // TODO: characterize flex sensors and actually fill this out
  return ((double) flex_val) / 2;
}

void publishEstimate(ros::Publisher estimate_pub)
{
  // TODO: improve!  currently just averages values
  
  double sum = 0;
  for (int i=0; i<3; i++)
    for (int j=0; j<10; j++)
      sum += diam_estimates[i][j];
  double estimate = sum / 30;
 
  std_msgs::Float64 msg;
  msg.data = estimate;
  estimate_pub.publish(msg);
}

double getEstimate(int f)
{
  double sum=0;
  for (int i=0; i<10; i++)
    {
      sum += diam_estimates[f][i];
    }

  return sum / 10;
}

void sendCommand(const std_msgs::String::ConstPtr &cmsg)
{
  if(cmsg->data.compare("closeall") == 0)
    {
      baxter_soft_hand::finger_target msg;
      msg.finger = 1; // bottom finger of the triple
      msg.target = 3100;
      target_pub.publish(msg);
      msg.finger = 2; // middle finger of the triple
      msg.target = 3000;
      target_pub.publish(msg);
      msg.finger = 3; // top finger of the triple
      msg.target = 2500;
      target_pub.publish(msg);
      msg.finger = 4; // ''thumb''
      msg.target = 2800;
      target_pub.publish(msg);
    }
  else if(cmsg->data.compare("closethree") == 0)
    {
      baxter_soft_hand::finger_target msg;
      msg.finger = 1; // bottom finger of the triple
      msg.target = 3100;
      target_pub.publish(msg);
      //msg.finger = 2; // middle finger of the triple
      //msg.target = 3000;
      //target_pub.publish(msg);
      msg.finger = 3; // top finger of the triple
      msg.target = 2500;
      target_pub.publish(msg);
      msg.finger = 4; // ''thumb''
      msg.target = 2800;
      target_pub.publish(msg);
    }
  else if(cmsg->data.compare("closetwo") == 0)
    {
      baxter_soft_hand::finger_target msg;
      //msg.finger = 1; // bottom finger of the triple
      //msg.target = 3100;
      //target_pub.publish(msg);
      msg.finger = 2; // middle finger of the triple
      msg.target = 3000;
      target_pub.publish(msg);
      //msg.finger = 3; // top finger of the triple
      //msg.target = 2500;
      //target_pub.publish(msg);
      msg.finger = 4; // ''thumb''
      msg.target = 2800;
      target_pub.publish(msg);
    }
  else if (cmsg->data.compare("open") == 0)
    {
      baxter_soft_hand::finger_target msg;
      msg.finger = 1;
      msg.target = 500;
      target_pub.publish(msg);
      msg.finger = 2;
      msg.target = 500;
      target_pub.publish(msg);
      msg.finger = 3;
      msg.target = 500;
      target_pub.publish(msg);
      msg.finger = 4;
      msg.target = 500;
      target_pub.publish(msg);
    }
  else if (cmsg->data.compare("half") == 0)
    {
      baxter_soft_hand::finger_target msg;
      /*msg.finger = 1;
      msg.target = 1750;
      target_pub.publish(msg);
      msg.finger = 2;
      msg.target = 1750;
      target_pub.publish(msg);
      msg.finger = 3;
      msg.target = 1750;
      target_pub.publish(msg);*/
    }
  else if (cmsg->data.compare("up") == 0)
    {
      // baxter joint command
      std_msgs::String msg;
      msg.data = "up";
      baxter_pub.publish(msg);
    }
  else if (cmsg->data.compare("down") == 0)
    {
      // baxter joint command
      std_msgs::String msg;
      msg.data = "down";
      baxter_pub.publish(msg);
    }
  else if (cmsg->data.compare("one") == 0)
    {
      // baxter joint command
      std_msgs::String msg;
      msg.data = "one";
      baxter_pub.publish(msg);
    }
  else if (cmsg->data.compare("two") == 0)
    {
      // baxter joint command
      std_msgs::String msg;
      msg.data = "two";
      baxter_pub.publish(msg);
    }
  else if (cmsg->data.compare("three") == 0)
    {
      // baxter joint command
      std_msgs::String msg;
      msg.data = "three";
      baxter_pub.publish(msg);
    }
  else if (cmsg->data.compare("four") == 0)
    {
      // baxter joint command
      std_msgs::String msg;
      msg.data = "four";
      baxter_pub.publish(msg);
    }
  else if (cmsg->data.compare("top") == 0)
    {
      // baxter joint command
      std_msgs::String msg;
      msg.data = "top";
      baxter_pub.publish(msg);
    }
  else if (cmsg->data.compare("bottom") == 0)
    {
      // baxter joint command
      std_msgs::String msg;
      msg.data = "bottom";
      baxter_pub.publish(msg);
    }
  else if (cmsg->data.compare("print") == 0)
    {
      printf("\n");
      std::ofstream logfile;
      logfile.open("log15.txt", std::ios::app);
      for (int i=0; i<3; i++)
	{
	  logfile << getEstimate(i);
	  logfile << "\t";
	  ROS_INFO("finger num: %d", i+1);
	  ROS_INFO("value: %f", getEstimate(i));
	}
      logfile << "\n";
      logfile.close();
      printf("\n");

      baxter_soft_hand::flex_triple trip_msg;
      trip_msg.finger_1 = getEstimate(0);
      trip_msg.finger_2 = getEstimate(1);
      trip_msg.finger_3 = getEstimate(2);
      flex_triple_pub.publish(trip_msg);
    }
  else if (cmsg->data.compare("new") == 0)
    {
      std::ofstream logfile;
      logfile.open("log15.txt", std::ios::app);
      logfile << "new\n";
      logfile.close();
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Main_Grasp_Node");

  ros::NodeHandle n;

  ros::Subscriber flex_sub = n.subscribe("flex_sensor_val", 1000, getFlexVal);
  ros::Subscriber comm_sub = n.subscribe("command", 1000, sendCommand);
  ros::Publisher estimate_pub = n.advertise<std_msgs::Float64>("diameter_estimate", 1000);
  target_pub = n.advertise<baxter_soft_hand::finger_target>("finger_target", 1000);
  baxter_pub = n.advertise<std_msgs::String>("baxter_pos", 1000);
  flex_triple_pub = n.advertise<baxter_soft_hand::flex_triple>("flex_triple_from_grasp_control", 1000);  

  std::ofstream logfile;
  logfile.open("log15.txt");
  logfile << "test\n";
  logfile.close();

  while (ros::ok())
    {
      // calculate diameter estimate and publish it
      publishEstimate(estimate_pub);

      ros::Duration(.1).sleep();
      ros::spinOnce();
    }

  return 0;
}
