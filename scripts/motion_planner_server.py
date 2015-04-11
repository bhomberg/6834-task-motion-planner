#!/usr/bin/env python

import sys
import rospy
import re
import moveit_commander
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String


class MotionPlannerServer(object):
    def __init__(self, max_planning_time):
        self.max_planning_time = max_planning_time
        
    def get_motion_plan(self, req):
        print "============ Starting Moveit Commander"
        moveit_commander.roscpp_initialize(sys.argv)
        
        print "============ Setting current state"
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        
        world_start_state = req.parameters.state.world
        robot_start_state = req.parameters.state.robot
        action = re.split(',', req.parameters.action[1:-1])
        pose_goals = req.parameters.goals
    
        # TODO: Find alternate method for collision objects
        #scene.add_collision_objects(req.parameters.world.collision_objects)
        
        curr_state = robot_start_state
        res = motion_plan()
    
        print "============ Planning actoin ", action[0]
        for i in range(len(pose_goals)):
            print "============ Move group ", action[1]
            group = moveit_commander.MoveGroupCommander(action[1])
        
            group.set_planning_time(self.max_planning_time)
            group.set_start_state(curr_state)
            group.set_pose_target(pose_goals[i].pose)
        
            plan = group.plan()
    
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
            
            else:
                print "============ Component ", i, " of motion plan failed"
                res.state = req.parameters.state
                res.motion = [motion_seq()]
                res.success = False
                print "============ Done"
                return res   
    
        # Set state of world after action completion
        end_state = world_state()
        end_state.robot = curr_state
        
        obj_idx = self._search_for_object(action[2], world_start_state.collision_objects)
        
        end_state.world = world_start_state
        if obj_idx != -1:
            if action[0] == 'pickUp':
                end_state.world.collision_objects[obj_idx].primitive_poses[0] = pose_goals[-1].pose
            elif action[0] == 'putDown':
                end_state.world.collision_objects[obj_idx].primitive_poses[0] = pose_goals[1].pose
    
        res.state = end_state
        res.success = True        
        print "============ Done"
        return res
        
    def _search_for_object(self, obj_name, obj_list):
        for i in range(len(obj_list)):
            if obj_name == obj_list[i].id:
                return i                
        return -1
    
    def run(self):
        rospy.init_node('motion_planner_server')
        s = rospy.Service('motion_server_service', motion_service, self.get_motion_plan)
        print "============ Ready to serve motion plans"
        rospy.spin()
    
if __name__ == "__main__":
    motion_planner_server = MotionPlannerServer(1.0)
    motion_planner_server.run()
    
