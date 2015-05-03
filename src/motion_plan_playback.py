#!/usr/bin/env python

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
        self.planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene)
        self.objects = None
        self.left_gripper_state = None
        
        rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, self._update_world_state)
        rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self._update_left_gripper_state)
        
    def _search_for_object(self, obj_name, obj_list):
        for i in range(len(obj_list)):
            if obj_name == obj_list[i].id:
                return i                
        return -1    
        
    def _update_world_state(self, msg):
        collision_objects = msg.world.collision_objects
        attached_objects = []
        for obj in msg.robot_state.attached_collision_objects:
            attached_objects.append(obj.object)
            
        if not self.objects:
            if collision_objects and attached_objects:
                self.objects = collision_objects + attached_objects
            elif collision_objects:
                self.objects = collision_objects
            elif attached_objects:
                self.objects = attached_objects
                
        else:                
            # Update state of objects
            for idx,obj in enumerate(self.objects):
                if obj in collision_objects:
                    i = self._search_for_object(obj.id, collision_objects)
                    self.objects[idx] = collision_objects[i]
                    
                if obj in attached_objects:
                    i = self._search_for_object(obj.id, attached_objects)
                    self.objects[idx] = attached_objects[i]
        
    def _update_left_gripper_state(self, msg):
        self.left_gripper_state = msg.pose.position
        
    def _find_nearest_object(self):
        nearest_obj = None
        smallest_dist = float('inf')
        
        for obj in self.objects:
            gripper_x = self.left_gripper_state.x
            gripper_y = self.left_gripper_state.y
            gripper_z = self.left_gripper_state.z
            obj_x = obj.primitive_poses[0].position.x
            obj_y = obj.primitive_poses[0].position.y
            obj_z = obj.primitive_poses[0].position.z
            
            dist = ((gripper_x - obj_x)**2 + (gripper_y - obj_y)**2 + (gripper_z - obj_z)**2)**0.5
            if dist < smallest_dist:
                smallest_dist = dist
                nearest_obj = obj.id
                
        return nearest_obj
            
    def _playback(self, msg):
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface()
        
        # Get start state of world and robot
        world_start_state = msg.plan[0].state.world
        robot_start_state = msg.plan[0].state.robot
    
        # Set up moved group
        group = moveit_commander.MoveGroupCommander(robot_start_state.id)
    
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
        
        # Execute motion plan
        for step in msg.plan:
            for sub_step in step.motion:
                for traj in sub_step.trajectory.trajectory:
                    group.execute(traj)
                    rospy.sleep(2.0)
                nearest_obj = self._find_nearest_object()
                if sub_step.gripperOpen:
                    group.detach_object(nearest_obj)
                else:
                    group.attach_object(nearest_obj)
                
    def run(self):
        rospy.init_node('playback')
        bag = rosbag.Bag(self.filename)
        for topic, msg, t in bag.read_messages(topics=['plan']):
            self._playback(msg)
        bag.close()
        
if __name__ == "__main__":
    baxter = BaxterPlayback('/home/ragtz/test.bag')
    baxter.run()
                        
