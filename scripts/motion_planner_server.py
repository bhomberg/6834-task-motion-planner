#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String

def get_motion_plan(req):
    print "============ Starting Moveit Commander"
    moveit_commander.roscpp_initialize(sys.argv)
    
    print "============ Setting current state"
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    # TODO: Find alternate method for collision objects
    #scene.add_collision_objects(req.parameters.world.collision_objects)
    
    curr_state = req.parameters.start
    res = motion_plan()
    
    for i in range(len(req.parameters.group_names)):
        print "============ Move group ", req.parameters.group_names[0]
        group = moveit_commander.MoveGroupCommander(req.parameters.group_names[0])
        
        group.set_planning_time(5.0)
        group.set_start_state(curr_state)
        group.set_pose_target(req.parameters.goals[0])
        
        plan = group.plan()
    
        if len(plan.joint_trajectory.points) > 0 or len(plan.multi_dof_joint_trajectory.points) > 0:
            print "============ Motion plan succeeded"
            
            if i == 0:
                start = RobotState()
                start.joint_state.name = plan.joint_trajectory.joint_names
                start.joint_state.position = plan.joint_trajectory.points[0].positions
                start.joint_state.velocity = plan.joint_trajectory.points[0].velocities
                start.joint_state.effort = plan.joint_trajectory.points[0].effort
                #start.multi_dof_joint_state.joint_names = plan.multi_dof_joint_trajectory.joint_names
                #start.multi_dof_joint_state.transforms = plan.multi_dof_joint_trajectory.points[0].transforms
            
                res.start_state = start
                res.trajectory.trajectory_start = start 
            
            res.trajectory.trajectory.append(plan)
            
            ## Update current state for next planned action
            curr_state.joint_state.name = plan.joint_trajectory.joint_names
            curr_state.joint_state.position = plan.joint_trajectory.points[-1].positions
            curr_state.joint_state.velocity = plan.joint_trajectory.points[-1].velocities
            curr_state.joint_state.effort = plan.joint_trajectory.points[-1].effort
            #curr_state.multi_dof_joint_state.joint_names = plan.multi_dof_joint_trajectory.joint_names
            #curr_state.multi_dof_joint_state.transforms = plan.multi_dof_joint_trajectory.points[-1].transforms
            
            if i == len(req.parameters.group_names) - 1:
                res.end_state = curr_state
        else:
            print "============ Motion plan failed" 
            res.trajectory = DisplayTrajectory()
            res.success = False
            print "============ Done"
            return res   
    
    res.success = True        
    print "============ Done"
    return res
    
def motion_planner_server():
    rospy.init_node('motion_planner_server')
    s = rospy.Service('motion_server_service', motion_service, get_motion_plan)
    print "============ Ready to serve motion plans"
    rospy.spin()
    
if __name__ == "__main__":
    motion_planner_server()
    
