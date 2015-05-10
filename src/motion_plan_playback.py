#!/usr/bin/env python

import os
import sys
import rospy
import rosbag
import re
import moveit_commander
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from baxter_core_msgs.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String


DIR_6834 = os.path.abspath(os.path.dirname(__file__) + '/../') + '/'

def record(filename, start_state, plan, motion):
    bag = rosbag.Bag(filename, 'w')
    
    str_plan = []
    for step in plan:
        action = '('
        for elm in step:
            action += elm + ','
        action = action[:-1] + ')'
        str_plan.append(String(data=action))
    
    flat_motion = []
    for sub_motion in motion:
        flat_motion += sub_motion
    
    try:
        record = recorded_motion_plan()
        record.start_state = start_state
        record.plan = str_plan
        record.motion = flat_motion
        
        bag.write('plan', record)
    finally:
        bag.close()

class BaxterPlayback(object):    
    def __init__(self, filename):
        self.filename = filename
        self.planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene)
            
    def _playback(self, msg):
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface()
        
        # Get start state of world and robot
        world_start_state = msg.start_state.world
        robot_start_state = msg.start_state.robot
    
        group = moveit_commander.MoveGroupCommander('right_arm')#(robot_start_state.id)
    
        # Set up robot in start configuration
        curr_state = robot_start_state.state
        group.go(curr_state.joint_state)
        
        # Set up world in start configuration
        planning_scene_world = PlanningSceneWorld()
        for movable_object in world_start_state.movable_objects:
            planning_scene_world.collision_objects.append(movable_object)
        for surface in world_start_state.surfaces:
            planning_scene_world.collision_objects.append(surface)
        
        # Add objects to planning scene
        planning_scene = PlanningScene()
        planning_scene.world = planning_scene_world
        planning_scene.is_diff = True

        self.planning_scene_pub.publish(planning_scene)
        
        prev_gripper_state = True
        
        action_idx = -1
        for seq in msg.motion:
            for traj in seq.trajectory.trajectory:    
                group.execute(traj)
                rospy.sleep(5.0)
        
            if prev_gripper_state != seq.gripperOpen:
                action_idx += 1
                action = re.split(',', msg.plan[action_idx].data[1:-1])
                
                if seq.gripperOpen:
                    print "Open Gripper:", action[1]
                    group.detach_object(action[1])
                else:
                    print "Close Gripper:", action[1]
                    group.attach_object(action[1])
            
            prev_gripper_state = seq.gripperOpen
                
    def run(self):
        rospy.init_node('playback')
        bag = rosbag.Bag(self.filename)
        for topic, msg, t in bag.read_messages(topics=['plan']):
            self._playback(msg)
        bag.close()
        
if __name__ == "__main__":
    baxter = BaxterPlayback(DIR_6834+'playback/sort_test.bag')
    baxter.run()
                        
