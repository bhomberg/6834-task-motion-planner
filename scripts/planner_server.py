#!/usr/bin/env python

from task_motion_planner.srv import *
from task_motion_planner.msg import *
import rospy

def handle_pddl(req):
    print "got request"
    msg = hl_plan()
    msg.error = False
    return msg

def planner_server():
    rospy.init_node('planner_server')
    s = rospy.Service('task_server_service', task_service, handle_pddl)
    print "Ready to serve motion plans."
    rospy.spin()

if __name__ == "__main__":
    planner_server()
