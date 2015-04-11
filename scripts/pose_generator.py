#!/usr/bin/python

import roslib
import rospy
from geometry_msgs.msg import *
from task_motion_planner.msg import *
import math
import random
import copy

class PoseGenerator:
	# cylinder dimensions
	def __init__(self):
		self.table_height = .05
	
	def pickup(self,x,y,z,length,radius):
		# gripper distance from cylinder centerpoint during staging
		r = radius + .03 
		# height of gripper when grabbing
		z = self.table_height + .03

		# stage, generate a random pose on a larger circle around the cylinder
		poseGen1 = pose_gen()
		pose1 = poseGen1.pose
		pose1.position.y = random.uniform(y-r,y+r)
		pose1.position.x = math.sqrt(r**2 - (pose1.position.y - y)**2) - x
		pose1.position.z = z
		yaw = math.atan2(pose1.position.y/pose1.position.x)
		pose1.orientation = rpy2Orientation(0,0,yaw)
		poseGen1.gripperOpen = True

		#pre-grasp
		poseGen2 = pose_gen()
		pose2 = poseGen2.pose
		pose2.position.x = radius*(pose1.position.x/math.sqrt(pose1.position.x**2+pose1.position.y**2)) + x
		pose2.position.y = radius*(pose1.position.y/math.sqrt(pose1.position.x**2+pose1.position.y**2)) + y
		pose2.position.z = z
		pose2.orientation = pose1.orientation
		poseGen2.gripperOpen = True
		
		# grasp
		poseGen3 = pose_gen()
		pose3 = poseGen3.pose
		pose3.position.x = pose2.position.x
		pose3.position.y = pose2.position.y
		pose3.position.z = z
		pose3.orientation = pose2.orientation
		poseGen3.gripperOpen = False

		# lift above other cylinders
		poseGen4 = pose_gen()
		pose4 = poseGen4.pose
		pose4.position.x = pose2.position.x
		pose4.position.y = pose2.position.y
		pose4.position.z = table_height + .15
		pose4.orientation = pose3.orientation
		poseGen4.gripperOpen = False
		
		# move out of the way to a standard position
		poseGen5 = pose_gen()
		pose5 = poseGen5.pose
		pose5.position.z = table_height + .15
		pose5.orientation = rpy2Orientation(0,0,0)
		poseGen5.gripperOpen = False

		#Return array of custom pose messages - pose + boolean (open/closed)
		return [poseGen1,poseGen2,poseGen3,poseGen4,poseGen5] 

	def putdown(self,x1,y1,x2,y2):
		# sample a point inside the rectangle
		poseGen1 = pose_gen()
		poseGen1.pose.position.x = random.uniform(x1,x2)
		poseGen1.pose.position.y = random.uniform(y1,y2)
		poseGen1.pose.position.z = self.table_height + .15
		poseGen1.gripperOpen = False

		# set down
		poseGen2 = pose_gen()
		poseGen2.pose.position.x = poseGen1.pose.position.x
		poseGen2.pose.position.y = poseGen1.pose.position.y
		poseGen2.pose.position.z = self.table_height + .03
		poseGen2.pose.orientation = poseGen1.pose.orientation
		poseGen2.gripperOpen = False

		# open gripper
		poseGen3 = pose_gen()
		poseGen3.pose = poseGen2.pose
		poseGen3.gripperOpen = True

		# move back
		poseGen4 = pose_gen()
		poseGen4.pose.position.x = poseGen3.pose.position.x + .02
		poseGen4.pose.position.y = poseGen3.pose.position.y + .02
		poseGen4.pose.position.z = poseGen3.pose.position.z
		poseGen4.pose.orientation = poseGen3.pose.orientation
		poseGen4.gripperOpen = True
	    
	   	# move up
		poseGen5 = pose_gen()
		poseGen5.pose.position.x = poseGen4.pose.position.x
		poseGen5.pose.position.y = poseGen4.pose.position.y
		poseGen5.pose.position.z = poseGen5.pose.position.z + .10
		poseGen5.pose.orientation = poseGen4.pose.orientation
		poseGen5.gripperOpen = True
	        
		# move out of the way
		poseGen6 = pose_gen()
		poseGen6.pose.position.z = self.table_height + .10
		poseGen6.gripperOpen = True

		# Return array of custom pose messages - pose + boolean (open/closed)
		return [poseGen1,poseGen2,poseGen3,poseGen4,poseGen5,poseGen6]

	def rpy2Orientation(roll,pitch,yaw):
		c1 = math.cos(yaw/2)
		if pitch == math.pi/2:
			c2 = math.cos(math.pi/4)
			s2 = math.cos(math.pi/4)
		elif pitch == -math.pi/2:
			c2 = math.cos(-math.pi/4)
			s2 = math.sin(-math.pi/4)
		else:
			c2 = math.cos(pitch/2)
			s2 = math.sin(pitch/2)
		c3 = math.cos(roll/2)
		s1 = math.sin(yaw/2)
		s3 = math.sin(roll/2)

		result = Quaternion()
		result.x = s1 * s2 * c3 + c1 * c2 * s3
		result.y = s1 * c2 * c3 + c1 * s2 * s3
		result.z = c1 * s2 * c3 - s1 * c2 * s3
		result.w = c1 * c2 * c3 - s1 * s2 * s3
		return result

if __name__ == "__main__":
    poseGen = PoseGenerator()
    print poseGen.putdown(0,0,1,1)
