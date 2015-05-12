#!/usr/bin/env python

import sys
import rospy
import math
import re
import moveit_commander
import numpy as np
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String
from copy import deepcopy

z_off = 0.06
snp = 0.49999
ssp = -0.49999

b_dist = 0.7
b_width = 0.13

max_planner_repeats = 1

sort = True

class MotionPlannerServer(object):
    def __init__(self, max_planning_time, find_blocking_objects=False):
        self.max_planning_time = max_planning_time
        self.find_blocking_objects = find_blocking_objects
        self.planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene)
        self.collision_object_pub = rospy.Publisher('/collision_object', CollisionObject)
        
        self.attach_obj = None
        
    def get_motion_plan(self, req):
        world_start_state = req.parameters.state.world
        robot_start_state = req.parameters.state.robot
        print "PREACTION:", req.parameters.action
        action = re.split(',', req.parameters.action[1:-1])
        print "ACTION: ", action
        pose_goals = req.parameters.goals
        
        failed_res = motion_plan()
        failed_res.state = req.parameters.state
        failed_res.motion = [motion_seq()]
        failed_res.success = False
        
        if self.find_blocking_objects:
            print "============ Quick Collision Check"
            objects = world_start_state.movable_objects
            objects_copy = deepcopy(objects)
            for i, obj in enumerate(objects):
                if obj.id == action[1]:
                    objects_copy = objects[0:i] + objects[i+1:len(objects)]
                    break
            if sort:
                if action[0] == 'PICKUP':
                    blocking_objects = self._get_blocking_objects(pose_goals[1], objects_copy)
                else:
                    blocking_objects = self._get_blocking_objects(pose_goals[0], objects_copy)
            else:  
                blocking_objects = self._get_blocking_objects(pose_goals[1], objects_copy)
                
            if blocking_objects:
                print "============ Blocking Objects"
                print blocking_objects
                print "============ Done"
                return failed_res 
        
        for i in range(max_planner_repeats):
            (res, curr_state) = self._find_motion_plan(req)
            if res.success:
                break
        
        if not res.success:
            return failed_res
    
        # Set state of world after action completion
        end_state = world_state()
        end_state.robot = robot()
        end_state.robot.id = robot_start_state.id
        end_state.robot.state = curr_state
        
        mov_obj_idx = self._search_for_object(action[1], world_start_state.movable_objects)
        
        end_state.world = world_start_state
        if mov_obj_idx != -1:
            end_pose = Pose()
            
            if action[0] == 'PICKUP':
                end_pose.position.x = 0
                end_pose.position.y = 0
                end_pose.position.z = z_off
                end_pose.orientation.x = 0
                end_pose.orientation.y = -0.7071067811865475
                end_pose.orientation.z = 0
                end_pose.orientation.w = 0.7071067811865476
                end_state.world.movable_objects[mov_obj_idx].header.frame_id = '/right_gripper'
                #end_state.world.movable_objects[mov_obj_idx].header.frame_id = '/left_gripper'
            elif not sort and action[0] == 'PUTDOWN':
                x = pose_goals[1].pose.orientation.x
                y = pose_goals[1].pose.orientation.y
                z = pose_goals[1].pose.orientation.z
                w = pose_goals[1].pose.orientation.w
                (roll, pitch, yaw) = self._orientation_to_rpy(x, y, z, w)
                print roll, pitch, yaw
                end_pose.position.x = pose_goals[1].pose.position.x + z_off*math.cos(yaw)
                end_pose.position.y = pose_goals[1].pose.position.y - z_off*math.sin(yaw)
                end_pose.position.z = pose_goals[1].pose.position.z
                end_pose.orientation.w = 1
                end_state.world.movable_objects[mov_obj_idx].header.frame_id = '/base'
            
            end_state.world.movable_objects[mov_obj_idx].primitive_poses[0] = end_pose
            
            if sort and action[0] == 'PUTDOWN':
                idx = self._search_for_object(action[1], end_state.world.movable_objects)
                
                n = len(end_state.world.movable_objects)
                
                end_state.world.movable_objects = end_state.world.movable_objects[0:idx] + end_state.world.movable_objects[idx+1:n]
            
        res.state = end_state        
        print "============ Done"
        return res
    
    def _find_motion_plan(self, req):
        world_start_state = req.parameters.state.world
        robot_start_state = req.parameters.state.robot
        action = re.split(',', req.parameters.action[1:-1])
        pose_goals = req.parameters.goals
    
        res = motion_plan()
        
        print "============ Starting Moveit Commander"
        moveit_commander.roscpp_initialize(sys.argv)
        
        print "============ Setting current state"
        scene = moveit_commander.PlanningSceneInterface()
        
        # Set up moved group
        #group = moveit_commander.MoveGroupCommander(action[2])
        group = moveit_commander.MoveGroupCommander('right_arm')
        #group = moveit_commander.MoveGroupCommander('left_arm')
        
        # Set attach/detach point
        attach_detach_idx = 2
        
        # Clean up world
        if self.attach_obj:
            group.detach_object(self.attach_obj)
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        self.collision_object_pub.publish(co) 
        
        # Set up robot in start configuration
        curr_state = robot_start_state.state
        #group.go(curr_state.joint_state)
        
        # Wait for execution to complete
        #rospy.sleep(10.0)
        
        # Set up world
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
        
        # Attach grasped objects
        if action[0] == 'PUTDOWN':
            group.attach_object(action[1])            
    
        print "============ Planning action ", action[0]
        for i in range(len(pose_goals)):
            print "============ Move group ", action[2]
            #print "Setup planner\n"
            group.set_planning_time(self.max_planning_time)
            group.set_start_state(curr_state)
            group.set_pose_target(pose_goals[i].pose)
            #rospy.sleep(5.0)
            #print "Plan\n"
            plan = group.plan()
            #print "Plan Done\n"
            if len(plan.joint_trajectory.points) > 0:
                print "============ Component ", i, " of motion plan succeeded"
            
                # Determine start pose for trajectory
                start = RobotState()
                start.joint_state.name = plan.joint_trajectory.joint_names
                start.joint_state.position = plan.joint_trajectory.points[0].positions
                start.joint_state.velocity = plan.joint_trajectory.points[0].velocities
                start.joint_state.effort = plan.joint_trajectory.points[0].effort
            
                # Add trajectory plan to motion sequence list
                motion = motion_seq()
                motion.trajectory.trajectory_start = start 
                motion.trajectory.trajectory.append(plan)
                motion.gripperOpen = pose_goals[i].gripperOpen
            
                res.motion.append(motion)
            
                ## Update current state for next planned trajectory
                curr_state.joint_state.name = plan.joint_trajectory.joint_names
                curr_state.joint_state.position = plan.joint_trajectory.points[-1].positions
                curr_state.joint_state.velocity = plan.joint_trajectory.points[-1].velocities
                curr_state.joint_state.effort = plan.joint_trajectory.points[-1].effort
                
                #group.execute(plan)
                
                if action[0] == 'PICKUP' and i == attach_detach_idx:
                    self.attach_obj = action[1]
                    group.attach_object(action[1])
                elif action[0] == 'PUTDOWN' and i == attach_detach_idx:
                    group.detach_object(action[1])
                
                #rospy.sleep(2.0)
                #return
            else:
                print "============ Component ", i, " of motion plan failed"
                res.state = req.parameters.state
                res.motion = [motion_seq()]
                res.success = False
                
                print "============ Done" 
                return (res, curr_state)        
                
        res.success = True
        return (res, curr_state)
    
    def _orientation_to_rpy(self, x, y, z, w):
        test = x*y + z*w
        
        if test > snp or test < ssp:
            roll = 0
        else:
            roll = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
            
        if test > snp:
            pitch = math.pi/2
        elif test < ssp:
            pitch = -math.pi/2
        else:
            pitch = math.asin(2*(x*y - z*w))
            
        if test > snp:
            yaw = 2*math.atan2(y, w)
        elif test < ssp:
            yaw = -2*math.atan2(y, w)
        else:
            yaw = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)
            
        return (roll, pitch, yaw)
        
    def _search_for_object(self, obj_name, obj_list):
        for i in range(len(obj_list)):
            if obj_name == obj_list[i].id:
                return i                
        return -1
        
    def _get_blocking_objects(self, pose_goal, objects):
        x = pose_goal.pose.orientation.x
        y = pose_goal.pose.orientation.y
        z = pose_goal.pose.orientation.z
        w = pose_goal.pose.orientation.w
        (roll, pitch, yaw) = self._orientation_to_rpy(x, y, z, w)
                    
        goal = np.matrix([pose_goal.pose.position.x, pose_goal.pose.position.y])

        R = np.matrix([[math.cos(yaw), -math.sin(yaw)],
                       [math.sin(yaw),  math.cos(yaw)]])

        goal_p = R*goal.T + np.matrix([z_off,0]).T
                    
        p1 = goal_p + np.matrix([0,b_width/2.0]).T
        p2 = goal_p + np.matrix([-b_dist,-b_width/2.0]).T
                    
        blocking_objects = []
        for obj in objects:
            obj_pose = np.matrix([obj.primitive_poses[0].position.x, obj.primitive_poses[0].position.y])
                        
            obj_pose_p = R*obj_pose.T
                        
            if obj_pose_p[0,0] < p1[0,0] and obj_pose_p[0,0] > p2[0,0] and obj_pose_p[1,0] > p2[1,0] and obj_pose_p[1,0] < p1[1,0]:
                blocking_objects.append(obj.id)
                
        return blocking_objects
    
    def run(self):
        rospy.init_node('motion_planner_server')
        s = rospy.Service('motion_server_service', motion_service, self.get_motion_plan)
        print "============ Ready to serve motion plans"
        rospy.spin()
    
if __name__ == "__main__":
    motion_planner_server = MotionPlannerServer(10.0, True)
    motion_planner_server.run()
    
