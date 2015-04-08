#!/usr/bin/env python

from task_motion_planner.srv import *
from task_motion_planner.msg import *
import rospy
import os

def handle_pddl(req):
    print "Serving motion plan"
    domain_file = "domain"
    task_file = req.domain.task_file
    ff_call = "~/indigo_ws/src/6834-task-motion-planner/FF-v2.3/./ff -o " + domain_file + " -f " + task_file + " > output" 
    os.system(ff_call)

    msg = hl_plan()
    msg.error = True
    actions = False

    f = open('output')
    f2 = open('returnfile', 'w')
    for line in f:
        if line[0:4]=='step':
            msg.error = False
            actions = True
            f2.write(line[11:-1] + '\n')
        elif actions == True:
            if line == '\n':
                actions = False
            else:
                f2.write(line[11:-1] + '\n')
    
    f.close()
    f2.close()

    msg.file = "returnfile"
    return msg

def planner_server():
    rospy.init_node('planner_server')
    s = rospy.Service('task_server_service', task_service, handle_pddl)
    print "Ready to serve motion plans."
    rospy.spin()

if __name__ == "__main__":
    planner_server()
