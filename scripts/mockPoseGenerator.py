#!/usr/bin/python

import rospy
import roslib
from task_motion_planner.msg import *
from random import shuffle
import numpy
import re

# TODO: Change to message type instead of dict, also add second action

class MockPoseGenerator:
    def __init__(self):
        self.state = dict()

    def next(self, action_tuple):
        action = action_tuple[0]
        obj = action_tuple[1]

        msg = pose()
        msg.object_id = obj
        
        # TODO get surface/obj
        if action_tuple in self.state:
            # if there are letters left, return it
            if len(self.state[action_tuple]) > 0:
                if action == 'PICKUP':
                    msg.direction = self.state[action_tuple].pop()
                else:
                    location = self.state[action_tuple].pop()
                    msg.x = location[0]
                    msg.y = location[1]
                    msg.surface_id = action_tuple[-1]
                return msg
            else:
                return None
        else:
            # generate list of poses
            if action == 'PICKUP':
                self.state[action_tuple] = ['N','NE','E','SE','S','SW','W','NW']
                #shuffle(self.state[action_tuple])
                msg.direction = self.state[action_tuple].pop()
            else:
                x_poses =  list(xrange(17))
                y_poses = list(xrange(17))
                #shuffle(x_poses)
                #shuffle(y_poses)
                self.state[action_tuple]=numpy.transpose([x_poses,y_poses]).tolist()
                location = self.state[action_tuple].pop()
                msg.x = location[0]
                msg.y = location[1]
                msg.surface_id = action_tuple[-1]
            # return a pose
            return msg

    def reset(self, action):
        if action in self.state:
            del self.state[action]

    def resetAll(self):
        for action in self.state.keys():
            del self.state[action]

if __name__ == "__main__":
    m = MockPoseGenerator()
    for i in range(9):
        print m.next('(PICKUP,1)')
    for i in range(18):
        print m.next('(PUTDOWN,1,surface)')
    m.reset('PICKUP')
    m.reset('PUTDOWN')
    print m.next('(PICKUP,1)')
    print m.next('(PUTDOWN,1,surface)')
