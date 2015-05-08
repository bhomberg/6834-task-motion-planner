#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../src/'))
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from mockStateUpdate import *
from interface_layer_organized import *
from pose_generator import *
from baxter_small_tests import *
from std_msgs.msg import *
from moveit_msgs.msg import *
from shape_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from motion_plan_playback import *


DIR_6834 = os.path.abspath(os.path.dirname(__file__) + '/../') + '/'

if __name__ == '__main__':
    poseGen = PoseGenerator()
    
    filepath = DIR_6834+"states/SQUARE_3"	
    f = open(filepath,'r')
    init_state_string = f.read()
    state = [[]]*3
    l = init_state_string.split('\n')
    state[0] = l[0].split(',')
    k = l[1].split(',')
    state[1] = [tuple(i.split(' ')) for i in k]
    k = l[2].split(',')
    state[2] = [tuple(i.split(' ')) for i in k]
    pose = l[3]
	
    world = makeState('SQUARE')
    
    interfaceLayer = InterfaceLayer('task_server_service', 'motion_server_service', poseGen, mockStateUpdate, DIR_6834)
    (plan, traj) = interfaceLayer.run(state, world, pose)
    
    record(DIR_6834+'playback/square_test.bag', world, plan, traj)
    
