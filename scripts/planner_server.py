#!/usr/bin/env python

from task_motion_planner.srv import *
from task_motion_planner.msg import *
import rospy
import os

class TaskPlannerServer(object):
    def __init__(self, path, domain_file):
        self.path = path
        self.domain_file = domain_file
        
    def handle_pddl(self, req):
        print "Serving motion plan"
        domain_file = "domain"
        task_file = req.domain.task_file
        ff_call = self.path + "./ff -o " + self.domain_file + " -f " + task_file + " > output" 
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

    def run():
        rospy.init_node('task_planner_server')
        s = rospy.Service('task_server_service', task_service, self.handle_pddl)
        print "Ready to serve task plans"
        rospy.spin()

if __name__ == "__main__":
    task_planner_server = TaskPlannerServer("~/indigo_ws/src/6834-task-motion-planner/FF-v2.3/", "domain")
    task_planner_server.run()
