#!/usr/bin/python

import roslib
import rospy
from geometry_msgs.msg import *
from task_motion_planner.msg import *
import math
import random
import copy
import re

BOUND = math.pi/2.0

# Generates a set of gripper poses given an action and a world description
# The motion planner verifies that the set of candidate poses is valid 
# (not obstructed by objects & objects are reachable) 
class PoseGenerator:

    def __init__(self, SLICES=1, GRIPPER_OFFSET = .015):
        self.pickup_counter = 0
        self.putdown_counter = 0
        # the height of the table in world coordinates
        self.DIST_FROM_CYLINDER = .1
        self.GRIPPER_OFFSET = GRIPPER_OFFSET
        self.SLICES = SLICES
        self.sliceSize = 2 * math.pi/self.SLICES
        self.pickup_ub = -BOUND
        self.putdown_ub = -BOUND
        self.MAX_COUNT = SLICES

    # Generates a gripper pose given an action and a world description
    # action = a string containing (action, arm, object_name)
    # world = a WorldState msg
    # return = a gripper pose, which is a set of waypoints
    def next(self, action, world):
        if isinstance(action, tuple):
            a = '('
            for elm in action:
                a += elm + ','
            action = a[:-1] + ')'
        
        action = re.split(',', action[1:-1])
        objects = world.world.movable_objects
        surfaces = world.world.surfaces
        print "action[1]:", action[1]
        if action[1] == ' ':
            return None
        obj = self._search_for_object(action[1], objects)
        height = obj.primitives[0].dimensions[0]
        radius = obj.primitives[0].dimensions[1]
        
        if action[0] == 'PICKUP':
            if self.pickup_counter < self.MAX_COUNT:
                self.pickup_counter += 1
                pose = obj.primitive_poses[0]
                return self.pickup(pose,height,radius)
        elif action[0] == 'PUTDOWN':
            if self.putdown_counter < self.MAX_COUNT:
                self.putdown_counter += 1
                table = self._search_for_object(action[-1], surfaces)
                return self.putdown(table,height,radius)
        return None

    def reset(self,action):
        if action == 'PICKUP':
            self.pickup_counter = 0
        elif action == 'PUTDOWN':
            self.putdown_counter = 0
    
    def resetAll(self):
        self.putdown_counter = 0
        self.pickup_counter = 0

    # Generates a gripper pose for a pickup action of a cylinder
    # obj_pose = cylinder pose (position, orientation)
    # height = cylinder height
    # radius = cylinder radius
    # return =  a list of 5 pose messages each containing a waypoint and a
    #           boolean - true if the gripper is open, false if closed
    #           poses = stage, pre-grasp, grasp, lifted, standard pose
    def pickup(self, obj_pose, height, radius):
        CLEARANCE_HEIGHT = obj_pose.position.z + height
        
        # lower bound is equal to the previous upper bound
        if self.pickup_ub == BOUND:
            self.pickup_ub = -BOUND #reset
        self.pickup_lb = self.pickup_ub
        self.pickup_ub += self.sliceSize
        print self.pickup_lb, self.pickup_ub
        
        # radius of circle around the cylinder where the gripper origin will lie
        r = radius + self.DIST_FROM_CYLINDER
        # height of gripper when grasping cylinder
        z =  obj_pose.position.z

        # A random pose lying on a circle around the cylinder
        # pointing towards the cylinder with the gripper open
        poseGen1 = pose()
        pose1 = poseGen1.pose
        # random yaw position
        yaw = random.uniform(self.pickup_lb, self.pickup_ub)
        # x,y position along a circle around the cylinder
        pose1.position.x = obj_pose.position.x + r * math.cos(yaw - math.pi)
        pose1.position.y = obj_pose.position.y - r * math.sin(yaw - math.pi)
        pose1.position.z = z
        pose1.orientation = self._rpy_to_orientation(math.pi/2.0,0,yaw)
        # yaw position s.t. the gripper points towards the cylinder
        poseGen1.gripperOpen = True

        # Pre-grasp: A pose s.t. the open gripper is touching the cylinder
        poseGen2 = pose()
        pose2 = poseGen2.pose
        # x,y pose s.t. gripper moves towards the cylinder and touches it
        pose2.position.x = (radius / r) * (pose1.position.x - obj_pose.position.x)*3.0 + obj_pose.position.x
        pose2.position.y = (radius / r) * (pose1.position.y - obj_pose.position.y)*3.0 + obj_pose.position.y
        pose2.position.z = z
        pose2.orientation = pose1.orientation
        poseGen2.gripperOpen = True
        
        # Grasp: A pose that grasps the cylinder
        poseGen3 = pose()
        poseGen3.pose = pose2
        poseGen3.gripperOpen = False

        # Standard Pose: A pose s.t. the group is out of the way of the other 
        # objects in a standard position
        poseGen4 = pose()
        poseGen4.pose.position.x = .8
        poseGen4.pose.position.y = .8
        poseGen4.pose.position.z = CLEARANCE_HEIGHT
        poseGen4.pose.orientation = self._rpy_to_orientation(math.pi/2.0,0,0)
        poseGen4.gripperOpen = False

        # An array of pose messages
        return [poseGen1,poseGen2,poseGen3,poseGen4] 

    # Generates a set of gripper poses for a putting down a cylinder,
    # given an area in which to place the object
    # x1, y1 = bottom left corner of area (from top view)
    # x2, y2 = top right corner of area (from top view)
    # return =  a list of 6 pose messages containing a waypoint and a
    #           boolean - true if the gripper is open, false if closed
    #           waypoints = stage, set-down, let-go, back away, lift arm, standard pose
    def putdown(self,table,height,radius):
        # lower bound is equal to the previous upper bound
        if self.putdown_ub >= BOUND:
            self.putdown_ub = -BOUND #reset
        self.putdown_lb = self.putdown_ub
        self.putdown_ub += self.sliceSize
        print self.putdown_lb, self.putdown_ub

        table_center = table.primitive_poses[0].position
        table_height = table.primitive_poses[0].position.z + table.primitives[0].dimensions[2]/2.0
        x1 = table_center.x - table.primitives[0].dimensions[0]/2.0
        y1 = table_center.y - table.primitives[0].dimensions[1]/2.0
        x2 = table_center.x + table.primitives[0].dimensions[0]/2.0
        y2 = table_center.y + table.primitives[0].dimensions[1]/2.0
        CLEARANCE_HEIGHT = table_height + height #note: this is less clearance than before, but means that our sequence of poses are more likely to be feasible
        r = radius + self.DIST_FROM_CYLINDER

        # Generate a pose hovering over a sampled (x,y) point
        poseGen1 = pose()
        poseGen1.pose.position.x = random.uniform(x1,x2)
        poseGen1.pose.position.y = random.uniform(y1,y2)
        # print "(x,y): ", (poseGen1.pose.position.x, poseGen1.pose.position.y)
        poseGen1.pose.position.z = CLEARANCE_HEIGHT
        yaw = random.uniform(self.putdown_lb,self.putdown_ub)
        poseGen1.pose.orientation = self._rpy_to_orientation(math.pi/2.0,0,yaw)
        poseGen1.gripperOpen = False

        # Set down
        poseGen2 = pose()
        poseGen2.pose.position.x = poseGen1.pose.position.x
        poseGen2.pose.position.y = poseGen1.pose.position.y
        poseGen2.pose.position.z = table_height + height/2.0
        poseGen2.pose.orientation = poseGen1.pose.orientation
        poseGen2.gripperOpen = False

        # Open gripper
        poseGen3 = pose()
        poseGen3.pose = poseGen2.pose
        poseGen3.gripperOpen = True

        # Move back
        poseGen4 = pose()
        poseGen4.pose.position.x = poseGen3.pose.position.x + .1* math.cos(yaw - math.pi)
        poseGen4.pose.position.y = poseGen3.pose.position.y - .1* math.sin(yaw - math.pi)
        poseGen4.pose.position.z = poseGen3.pose.position.z
        poseGen4.pose.orientation = poseGen3.pose.orientation
        poseGen4.gripperOpen = True
            
        # Move out of the way to the standard position
        poseGen5 = pose()
        poseGen5.pose.position.x = .6
        poseGen5.pose.position.y = .6
        poseGen5.pose.position.z = CLEARANCE_HEIGHT
        poseGen5.pose.orientation = self._rpy_to_orientation(math.pi/2.0,0,0)
        poseGen5.gripperOpen = True

        # Return array of custom pose messages
        return [poseGen1,poseGen2,poseGen3,poseGen4]#,poseGen5]

    # Gets an objects index from a list given the object's name
    # obj_name = object's name
    # obj_list = list of objects to search
    def _search_for_object(self, obj_name, obj_list):
        for i in range(len(obj_list)):
            #print "obj_name:", obj_name
            #print "obj_list[i].id:", obj_list[i].id + '\n'
            if obj_name == obj_list[i].id:
                return obj_list[i]              
        return None

    # Calculates the quaternion orientation given the roll, pitch, and yaw
    def _rpy_to_orientation(self, roll, pitch, yaw):
        # bank, attitude, heading
        c1 = math.cos(roll/2.0)
        s1 = math.sin(roll/2.0)
        c2 = math.cos(pitch/2.0)
        s2 = math.sin(pitch/2.0)
        c3 = math.cos(yaw/2.0)
        s3 = math.sin(yaw/2.0)

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
    # print poseGen.next('PUTDOWN')
