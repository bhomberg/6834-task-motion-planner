#!/usr/bin/python

import roslib
import rospy
from geometry_msgs.msg import *
from task_motion_planner.msg import *
import math
import random
import copy
import re

#TODO: right/left arm ---> standard pose is different

# Generates a set of gripper poses given an action and a world description
# The motion planner verifies that the set of candidate poses is valid 
# (not obstructed by objects & objects are reachable) 
class PoseGenerator:
    def __init__(self, table_height=0.25, GRIPPER_OFFSET = .015):
        # the height of the table in world coordinates
        self.table_height = table_height
        self.GRIPPER_OFFSET = GRIPPER_OFFSET

    # Generates a set of gripper poses given an action and a world description
    # action = a string containing (action, arm, object_name)
    # world = a WorldState msg
    def generate(self, action, world):
        action = re.split(',', action[1:-1])
        obj_name = action[-1]
        objects = world.world.collision_objects
        obj_idx = self._search_for_object(obj_name, objects)
        obj = objects[obj_idx]
        height = obj.primitives[0].dimensions[0]
        radius = obj.primitives[0].dimensions[1]
        pose = obj.primitive_poses[0]

        if action[0] == 'pickup':
            return self.pickup(pose,height,radius)
        elif action[0] == 'putdown':
            pass
        else:
            return

    # Generates a set of gripper poses for a pickup action of a cylinder
    # obj_pose = cylinder pose (position, orientation)
    # height = cylinder height
    # radius = cylinder radius
    # return =  an array of 5 pose_gen messages each containing a pose and a
    #           boolean - true if the gripper is open, false if closed
    #           poses = stage, pre-grasp, grasp, lifted, standard pose
    def pickup(self, obj_pose, height, radius):
        CLEARANCE_HEIGHT = .15
        DIST_FROM_CYLINDER = .1
        
        # radius of circle around the cylinder where the gripper origin will lie
        r = radius + DIST_FROM_CYLINDER
        # height of gripper when grasping cylinder
        z =  obj_pose.position.z

        # A random pose lying on a circle around the cylinder
        # pointing towards the cylinder with the gripper open
        poseGen1 = pose_gen()
        pose1 = poseGen1.pose
        # random y position at a set distance from the cylinder
        pose1.position.y = random.uniform(obj_pose.position.y-r, obj_pose.position.y+r)
        # TODO: multiply -1 or 1 by the sqrt? -- would need orientation
        # multiplier = 1 if random.random() < .5 else -1
        # x position along the circle around the cylinder
        pose1.position.x = obj_pose.position.x - math.sqrt(r**2 - (pose1.position.y - obj_pose.position.y)**2)
        pose1.position.z = z
        # yaw position s.t. the gripper points towards the cylinder
        yaw = -math.atan2(obj_pose.position.y-pose1.position.y, (obj_pose.position.x-(pose1.position.x-self.GRIPPER_OFFSET)))
        pose1.orientation = self._rpy_to_orientation(math.pi/2,0,yaw)
        poseGen1.gripperOpen = True

        # Pre-grasp: A pose s.t. the open gripper is touching the cylinder
        poseGen2 = pose_gen()
        pose2 = poseGen2.pose
        # x,y pose s.t. gripper moves towards the cylinder and touches it
        pose2.position.x = radius / r * (pose1.position.x - 15*self.GRIPPER_OFFSET - obj_pose.position.x) + obj_pose.position.x
        pose2.position.y = radius / r * (pose1.position.y - obj_pose.position.y) + obj_pose.position.y
        pose2.position.z = z
        pose2.orientation = pose1.orientation
        poseGen2.gripperOpen = True
        
        # Grasp: A pose that grasps the cylinder
        poseGen3 = pose_gen()
        poseGen3.pose = pose2
        poseGen3.gripperOpen = False

        # A pose s.t. the cylinder is lifted above all the other cylinders
        poseGen4 = pose_gen()
        pose4 = poseGen4.pose
        pose4.position.x = pose2.position.x
        pose4.position.y = pose2.position.y
        pose4.position.z = self.table_height + CLEARANCE_HEIGHT
        pose4.orientation = poseGen3.pose.orientation
        poseGen4.gripperOpen = False
        
        # Standard Pose: A pose s.t. the group is out of the way of the other 
        # objects in a standard position
        poseGen5 = pose_gen()
        pose5 = poseGen5.pose
        # x,y = 0,0
        pose5.position.z = self.table_height + CLEARANCE_HEIGHT
        pose5.orientation = self._rpy_to_orientation(math.pi/2,0,0)
        poseGen5.gripperOpen = False

        # An array of pose_gen messages
        # return [poseGen1]
        return [poseGen1,poseGen2,poseGen3,poseGen4,poseGen5] 

    # Generates a set of gripper poses for a putting down a cylinder,
    # given an area in which to place the object
    # x1, y1 = top left corner of area (from top view)
    # x2, y2 = botttom right corner of area (from top view)
    # return =  an array of 6 pose_gen messages containing a pose and a
    #           boolean - true if the gripper is open, false if closed
    #           poses = stage, set-down, let-go, back away, lift arm, standard pose
    def putdown(self,x1,y1,x2,y2):
        CLEARANCE_HEIGHT = .15
        HEIGHT_ABOVE_TABLE = .03
        X_Y_DIST_FROM_CYLINDER = .05
        
        # Sample an (x,y) point inside the given area
        # Generate a pose hovering over the point
        poseGen1 = pose_gen()
        poseGen1.pose.position.x = random.uniform(x1,x2)
        poseGen1.pose.position.y = random.uniform(y1,y2)
        poseGen1.pose.position.z = self.table_height + CLEARANCE_HEIGHT
        yaw = random.uniform(-math.pi/4,math.pi/4)
        poseGen1.pose.orientation = _rpy_to_orientation(math.pi/2,0,yaw)
        poseGen1.gripperOpen = False

        # Set down pose
        poseGen2 = pose_gen()
        poseGen2.pose.position.x = poseGen1.pose.position.x
        poseGen2.pose.position.y = poseGen1.pose.position.y
        poseGen2.pose.position.z = self.table_height + HEIGHT_ABOVE_TABLE
        poseGen2.pose.orientation = poseGen1.pose.orientation
        poseGen2.gripperOpen = False

        # Open gripper
        poseGen3 = pose_gen()
        poseGen3.pose = poseGen2.pose
        poseGen3.gripperOpen = True

        # Move back pose
        poseGen4 = pose_gen()
        poseGen4.pose.position.x = poseGen3.pose.position.x + X_Y_DIST_FROM_CYLINDER
        poseGen4.pose.position.y = poseGen3.pose.position.y + X_Y_DIST_FROM_CYLINDER
        poseGen4.pose.position.z = poseGen3.pose.position.z
        poseGen4.pose.orientation = poseGen3.pose.orientation
        poseGen4.gripperOpen = True
        
        # Move up (over the cylinders) pose
        poseGen5 = pose_gen()
        poseGen5.pose.position.x = poseGen4.pose.position.x
        poseGen5.pose.position.y = poseGen4.pose.position.y
        poseGen5.pose.position.z = poseGen5.pose.position.z + CLEARANCE_HEIGHT
        poseGen5.pose.orientation = poseGen4.pose.orientation
        poseGen5.gripperOpen = True
            
        # Move out of the way to the standard position
        poseGen6 = pose_gen()
        poseGen6.pose.position.z = self.table_height + CLEARANCE_HEIGHT
        poseGen6.pose.orientation = poseGen5.pose.orientation
        poseGen6.gripperOpen = True

        # Return array of custom pose messages
        return [poseGen1,poseGen2,poseGen3,poseGen4,poseGen5,poseGen6]

    # Gets an objects index from a list given the object's name
    # obj_name = object's name
    # obj_list = list of objects to search
    def _search_for_object(self, obj_name, obj_list):
        for i in range(len(obj_list)):
            if obj_name == obj_list[i].id:
                return i                
        return -1

    # Calculates the quaternion orientation given the roll, pitch, and yaw
    def _rpy_to_orientation(self, roll, pitch, yaw):
        # bank, attitude, heading
        c1 = math.cos(roll/2)
        s1 = math.sin(roll/2)
        c2 = math.cos(pitch/2)
        s2 = math.sin(pitch/2)
        c3 = math.cos(yaw/2)
        s3 = math.sin(yaw/2)

        result = Quaternion()
        result.x = s1 * s2 * c3 + c1 * c2 * s3
        result.y = s1 * c2 * c3 + c1 * s2 * s3
        result.z = c1 * s2 * c3 - s1 * c2 * s3
        result.w = c1 * c2 * c3 - s1 * s2 * s3
        return result

if __name__ == "__main__":
    poseGen = PoseGenerator()
    # test case
    # yaw, roll, pitch
    print poseGen._rpy_to_orientation(-math.pi/4,0,math.pi/2)
    print poseGen._rpy_to_orientation(0,0,math.pi/4)
