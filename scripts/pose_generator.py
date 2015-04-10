#!/usr/bin/python
from pose import Pose

import math
import random


def pickup(x,y,z,length,radius):
	# cylinder dimensions
	table_height = .05
	# gripper distance from cylinder centerpoint during staging
	r = radius + .03 
	# height of gripper when grabbing
	z = table_height + .03

	#stage
	# random number in the range x-r, x+r
	pose1 = Pose()
	pose1.x = random.uniform(x-r,x+r)
	pose1.y = math.sqrt(r**2 - (pose_x - x)**2) + y
	pose1.z = z
	pose1.gripperOpen = True

	# pre-grasp
	pose2 = pose1.clone()
	pose2.x = radius*(pose_x/math.sqrt(pose_x**2+pose_y**2))
	pose2.y = radius*(pose_y/math.sqrt(pose_x**2+pose_y**2))
	
	# grasp
	pose3 = pose2.clone()
	pose3.gripperOpen = False

	# lift above other cylinders
	pose4 = pose2.clone()
	pose4.z = table_height + .10
	pose4.gripperOpen = False
	
	# move out of the way to a standard position
	pose5 = Pose(0, 0, table_height + .15, False)

	#Return array of custom pose messages - pose + boolean (open/closed)
	return [pose1,pose2,pose3,pose4,pose5] 

def putdown(x1,y1,x2,y2):
	table_height = .05
	# sample a point
	pose1 = Pose()
	pose1.x = random.uniform(x1,x2)
	pose1.y = random.uniform(y1,y2)
	pose1.z = table_height + .10
	pose1.gripperOpen = False

	# set down
	pose2 = pose1.clone()
	pose2.z = table_height + .03
	pose2.gripperOpen = False

	# open gripper
	pose3 = pose2.clone()
	pose3.gripperOpen = True

	# move back
	pose4 = pose2.clone()
	pose4.x = pose4.x + .02
	pose4.y = pose4.y + .02
	pose4.gripperOpen = True
        
	# move out of the way
	pose5 = Pose()
	pose5.x = 0
	pose5.y = 0
	pose5.z = table_height + .10
	pose5.gripperOpen = True

	# Return array of custom pose messages - pose + boolean (open/closed)
	return [pose1,pose2,pose3,pose4,pose5]
