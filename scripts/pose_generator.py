#!/usr/bin/python

import roslib
roslib.load_manifest("6834-task-motion-planner")
import geometry_msgs
from geometry_msgs.msg import *
from task_motion_planner.msg import *

import rospy
import math
import random


def pickup(msg):
	# cylinder dimensions
	table_height = .05
	# gripper distance from cylinder centerpoint during staging
	r = msg.cylinder.radius + .03 
	# height of gripper when grabbing
	z = table_height + .03

	#stage
	pose = geometry_msgs.msg.Pose()
	pose.position.x = random.randint(msg.location.x - r, msg.location.x + r)
	pose.position.y = sqrt(r^2 - (x - msg.location.x)^2) + msg.location.y
	pose.position.z = z

	poseGen = pose_gen()
	poseGen.pose = pose
	poseGen.gripperOpen = True

	# pre-grasp
	pose2 = geometry_msgs.msg.Pose()
	pose2.position.x = msg.cylinder.radius*(msg.location.x/math.sqrt(msg.location.x^2+msg.location.y^2))
	pose2.position.y = msg.cylinder.radius*(msg.location.y/math.sqrt(msg.location.x^2+msg.locationy^2))
	pose2.position.z = z

	poseGen2 = pose_gen()
	poseGen2.pose = poseGen.pose
	poseGen2.gripperOpen = True
	
	# grasp
	poseGen3 = pose_gen()
	posGen3.pose = poseGen2.pose 
	poseGen3.gripperOpen = False

	# lift above other cylinders
	poseGen4 = pos_gen()
	poseGen4.pose = poseGen2.pose
	poseGen4.pose.position.z = table_height + .10
	poseGen4.gripperOpen = False
	
	# move out of the way to a standard position
	poseGen5 = geometry_msgs.msg.Pose()
	poseGen5.pose.position.x = 0
	poseGen5.pose.position.y = 0
	poseGen5.pose.position.z = table_height + .10
	poseGen5.gripperOpen = False

	#Return array of custom pose messages - pose + boolean (open/closed)
	return [poseGen1,poseGen2,poseGen3,poseGen4,poseGen5] 

def putdown(putdown_location):
	table_height = .05
	# sample a point
	poseGen1 = pos_gen()
	poseGen1.pose = geometry_msgs.msg.Pose()
	poseGen1.pose.x = random.randint(putdown_location.x1,putdown_location.x2)
	poseGen1.pose.y = random.randint(putdown_location.y1,putdown_location.y2)
	poseGen1.pose.z = table_height + .10
	poseGen1.gripperOpen = False
	# set down
	poseGen2 = pos_gen()
	poseGen2.pose = poseGen1.pose
	poseGen2.pose.z = table_height + .03
	poseGen2.gripperOpen = False

	# open gripper
	poseGen3.pose = poseGen2.pose
	poseGen3.gripperOpen = True

	# move back
	poseGen4.pose = poseGen2.pose
	poseGen4.pose.x = poseGen4.pose.x + 2;
	poseGen4.pose.x = poseGen4.pose.y + 2;
	poseGen3.gripperOpen = True
	# move out of the way
	poseGen5 = geometry_msgs.msg.Pose()
	poseGen5.pose.position.x = 0
	poseGen5.pose.position.y = 0
	poseGen5.pose.position.z = table_height + .10
	poseGen5.gripperOpen = True

	# Return array of custom pose messages - pose + boolean (open/closed)
	return [poseGen1,poseGen2,poseGen3,poseGen4,poseGen5]

def pose_generator():
    rospy.init_node('pose_generator')
    s = rospy.Service('pose_generator_service', pose_service, pickup)
    print "Ready to generate pose"
    rospy.spin()

if __name__ == "__main__":
    pose_generator()