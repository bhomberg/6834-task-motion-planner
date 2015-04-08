#!/usr/bin/env python

import sys
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *

def task_planner_client():
    rospy.wait_for_service('task_server_service')
    try:
        task_server = rospy.ServiceProxy('task_server_service', task_service)
        msg = task_domain()
        msg.task_file = 'problem0'
        resp1 = task_server(msg)
        return resp1.plan.error, resp1.plan.file
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print task_planner_client()
 
