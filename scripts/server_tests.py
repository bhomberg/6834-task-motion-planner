#!/usr/bin/env python

import sys
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *

def task_planner_client():
    rospy.wait_for_service('task_server_service')
    try:
        task_server = rospy.ServiceProxy('task_server_service', task_service)
        msg = task_domain()
        msg.task_file = 'problem0'
        resp = task_server(msg)
        return resp.plan.error, resp.plan.file
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def motion_planner_client():
    rospy.wait_for_service('motion_server_service')
    try:
        motion_server = rospy.ServiceProxy('motion_server_service', motion_service)
        
        msg = motion_plan_parameters()
        
        msg.world = PlanningSceneWorld()
        
        msg.start = RobotState()
        msg.start.joint_state.name = ['head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
        msg.start.joint_state.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg.start.joint_state.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg.start.joint_state.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg.group_names = ['left_arm']
        
        msg.goals = [Pose()]
        msg.goals[0].position.x = 0.74
        msg.goals[0].position.y = 0.28
        msg.goals[0].position.z = 0.34
        msg.goals[0].orientation.w = 1.0
        
        resp = motion_server(msg)
        
        print resp.plan.end_state
        
        return resp.plan.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    #print task_planner_client()
    print motion_planner_client()
 
