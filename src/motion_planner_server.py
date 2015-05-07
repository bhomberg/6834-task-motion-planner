#!/usr/bin/env python

import sys
import rospy
import math
import re
import moveit_commander
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String

z_off = 0.06
snp = 0.49999
ssp = -0.49999

b_dist = 0.3
b_width = 0.03

class MotionPlannerServer(object):
    def __init__(self, max_planning_time, find_blocking_objects=False):
        self.max_planning_time = max_planning_time
        self.find_blocking_objects = find_blocking_objects
        self.planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene)
        self.collision_object_pub = rospy.Publisher('/collision_object', CollisionObject)
        
        self.attach_obj = None
        
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
        #group = moveit_commander.MoveGroupCommander(action[2])
        group = moveit_commander.MoveGroupCommander('left_arm')
        
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
        
        res = motion_plan()
    
        print "============ Planning action ", action[0]
        for i in range(len(pose_goals)):
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
                
                if self.find_blocking_objects:
                    x = pose_goals[1].pose.orientation.x
                    y = pose_goals[1].pose.orientation.y
                    z = pose_goals[1].pose.orientation.z
                    w = pose_goals[1].pose.orientation.w
                    (roll, pitch, yaw) = self._orientation_to_rpy(x, y, z, w)
                    
                    x = pose_goals[1].pose.position.x
                    y = pose_goals[1].pose.position.y
                    
                    n1_x = math.cos(yaw)
                    n1_y = math.sin(yaw)
                    
                    n2_x = -math.sin(yaw)
                    n2_y = math.cos(yaw)
                    
                    x1 = x + z_off*math.cos(yaw)
                    y1 = y - z_off*math.sin(yaw)
                    
                    x2 = x1 - b_dist*math.cos(yaw)
                    y2 = y1 + b_dist*math.sin(yaw)
                    
                    x3 = x + b_width*math.cos(yaw - (math.pi/2))
                    y3 = y + b_width*math.sin(yaw - (math.pi/2))
                    
                    x4 = x + b_width*math.cos(yaw + (math.pi/2))
                    y4 = y + b_width*math.sin(yaw + (math.pi/2))
                    
                    print "x1:", x1
                    print "y1:", y1
                    print "x2:", x2
                    print "y2:", y2
                    print "x3:", x3
                    print "y3:", y3
                    print "x4:", x4
                    print "y4:", y4
                    
                    blocking_objects = []
                    for obj in world_start_state.movable_objects:
                        x = obj.primitive_poses[0].position.x
                        y = obj.primitive_poses[0].position.y
                        
                        print obj.id
                        print "c1:", y > (n1_y/n1_x)*(x - x1) + y1
                        print "c2:", y < (n1_y/n1_x)*(x - x2) + y2
                        print "c3:", y > (n2_y/n2_x)*(x - x3) + y3
                        print "c4:", y > (n2_y/n2_x)*(x - x4) + y4
                        
                        #if y > (n1_y/n1_x)*(x - x1) + y1 and y > (n2_y/n2_x)*(x - x2) + y2 and y > (n3_y/n3_x)*(x - x3) + y3 and y > (n4_y/n4_x)*(x - x4) + y4:
                        #    blocking_objects.append(obj)
                            
                    #print blocking_objects
                
                print "============ Done"
                return res   
    
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
                end_state.world.movable_objects[mov_obj_idx].header.frame_id = '/left_gripper'
            elif action[0] == 'PUTDOWN':
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
            
        res.state = end_state
        res.success = True        
        print "============ Done"
        return res
    
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
    
    def run(self):
        rospy.init_node('motion_planner_server')
        s = rospy.Service('motion_server_service', motion_service, self.get_motion_plan)
        print "============ Ready to serve motion plans"
        rospy.spin()
    
if __name__ == "__main__":
    motion_planner_server = MotionPlannerServer(10.0)
    motion_planner_server.run()
    
