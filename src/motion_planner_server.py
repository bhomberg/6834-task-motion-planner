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
        self.planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene)
        self.collision_object_pub = rospy.Publisher('/collision_object', CollisionObject)
        
        self.attach_obj = None
        self.objects = None
        rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, self._update_world_state)
        
    def get_motion_plan(self, req):
        print "============ Starting Moveit Commander"
        moveit_commander.roscpp_initialize(sys.argv)
        
        print "============ Setting current state"
        # robot = moveit_commander.RobotCommander
        scene = moveit_commander.PlanningSceneInterface()
        
        world_start_state = req.parameters.state.world
        robot_start_state = req.parameters.state.robot
        print "PREACTION:", req.parameters.action
        action = re.split(',', req.parameters.action[1:-1])
        print "ACTION: ", action
        pose_goals = req.parameters.goals
        
        # Set up moved group
        group = moveit_commander.MoveGroupCommander(action[2])
        
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
        group.go(curr_state.joint_state)
        
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
        
        res = motion_plan()
    
        print "============ Planning actoin ", action[0]
        for i in range(len(pose_goals)-1):
            print "============ Move group ", action[2]
        
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
                
                rospy.sleep(2.0)
                
                group.execute(plan)
                
                if action[0] == 'PICKUP' and i == attach_detach_idx:
                    self.attach_obj = action[1]
                    group.attach_object(action[1])
                elif action[0] == 'PUTDOWN' and i == attach_detach_idx:
                    group.detach_object(action[1])
                
                #return
                
            else:
                print "============ Component ", i, " of motion plan failed"
                res.state = req.parameters.state
                res.motion = [motion_seq()]
                res.success = False
                print "============ Done"
                return res   
    
        # Set state of world after action completion
        end_state = world_state()
        end_state.robot = robot()
        end_state.robot.id = robot_start_state.id
        end_state.robot.state = curr_state
        
        mov_obj_idx = self._search_for_object(action[1], world_start_state.movable_objects)
        obj_idx = self._search_for_object(action[1], self.objects)
        
        end_state.world = world_start_state
        if mov_obj_idx != -1 and obj_idx != -1:
            end_state.world.movable_objects[mov_obj_idx] = self.objects[obj_idx]
            #if action[0] == 'PICKUP':
            #    end_state.world.collision_objects[obj_idx].primitive_poses[0] = pose_goals[-1].pose
            #elif action[0] == 'putDown':
            #    end_state.world.collision_objects[obj_idx].primitive_poses[0] = pose_goals[1].pose
    
        res.state = end_state
        res.success = True        
        print "============ Done"
        return res
    
    def _update_world_state(self, msg):
        self.objects = msg.world.collision_objects
        
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
    motion_planner_server = MotionPlannerServer(5.0)
    motion_planner_server.run()
    
