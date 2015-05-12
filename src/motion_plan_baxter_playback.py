#!/usr/bin/env python

import sys
import rospy
import rosbag
import re
import baxter_interface
import moveit_commander
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from baxter_core_msgs.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String


def record(filename, plan):
    bag = rosbag.Bag(filename, 'w')
    
    try:
        record = recorded_motion_plan()
        record.plan = plan
        
        bag.write('plan', record)
    finally:
        bag.close()

class BaxterPlayback(object):    
    def __init__(self, filename):
        self.filename = filename
        
        #rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, self._update_world_state)
        #rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self._update_left_gripper_state)
        

            
    def _playback(self, msg):
        print "STARTED PLAYBACK"
        moveit_commander.roscpp_initialize(sys.argv)
        print "INITIALIZING GRIPPER"
        right = baxter_interface.Gripper('right')
        right.calibrate()
        print "ABOUT TO GET SCENE"
        scene = moveit_commander.PlanningSceneInterface()
        
        print "START STATE"
        # Get start state of world and robot
        world_start_state = msg.start_state.world
        robot_start_state = msg.start_state.robot
    
        # Set up moved group
        print "SET UP GROUP"
        group = moveit_commander.MoveGroupCommander('right_arm')#(robot_start_state.id)
        print "DONE"

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

        #self.planning_scene_pub.publish(planning_scene)
        
        # Execute motion plan
        for step in msg.motion:
            for traj in step.trajectory.trajectory:
                if step.gripperOpen:
                    right.open()
                else:
                    right.close()
                group.execute(traj)
                rospy.sleep(7.0)

    def run(self):
        rospy.init_node('playback')
        bag = rosbag.Bag(self.filename)
        print "GOT BAG"
        for topic, msg, t in bag.read_messages(topics=['plan']):
            self._playback(msg)
        bag.close()
        
if __name__ == "__main__":
    baxter = BaxterPlayback('/home/bhomberg/indigo_ws/src/6834-task-motion-planner/playback/sort_test_final.bag')
    baxter.run()
                        
