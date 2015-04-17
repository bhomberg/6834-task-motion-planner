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
        print "Serving task plan"
        domain_file = "domain"
        task_file = req.domain.task_file
        ff_call = self.path + "./ff -o " + self.domain_file + " -f " + task_file + " > output" 
        os.system(ff_call)

        msg = hl_plan()
        msg.error = True
        actions = False

        f = open('output')
        s = ""
        for line in f:
            if line[0:4]=='step':
                msg.error = False
                actions = True
                s = s + line[11:-1]
            elif actions == True:
                if line  == '\n':
                    actions = False
                else:
                    s = s + '\n' + line[11:-1]
        f.close()
        msg.plan = s
        #print "string: ", s

        return msg

    def run(self):
        rospy.init_node('task_planner_server')
        s = rospy.Service('task_server_service', task_service, self.handle_pddl)
        print "Ready to serve task plans"
        rospy.spin()

if __name__ == "__main__":
    DIR = "/home/vmlane/catkin_ws/src/6834-task-motion-planner"
    task_planner_server = TaskPlannerServer(DIR+"/FF-v2.3/", DIR + "/domain")
    task_planner_server.run()
    