#!/usr/bin/env python

from mockPoseGenerator import *
from mockMotionPlanner import *
import mockStateUpdate
from interface_layer_organized import *
from planner_server import *
from generateWorldMsg import *

DIR = '/home/vmlane/catkin_workspace/src/6834-task-motion-planner/'

if __name__ == '__main__':

    genWorld = generateWorldMsg()
    
    f = open(DIR+'states/one_cover','r')
    init_state_string = f.read()
    
    state = [[]]*3
    l = init_state_string.split('\n')
    state[0] = l[0].split(',')
    k = l[1].split(',')
    state[1] = [i.split(' ') for i in k]
    k = l[2].split(',')
    state[2] = [i.split(' ') for i in k]
    pose = l[3]

    world = genWorld.generateWorld('SQUARE',3)

    #task_planner_server = TaskPlannerServer(DIR+"/FF-v2.3/", DIR+"/domain")
    #task_planner_server.run()
    #motion_planner_server = MockMotionPlannerServer(17)
    #motion_planner_server.run()
    interfaceLayer = InterfaceLayer('task_server_service', 'motion_server_service', MockPoseGenerator, mockStateUpdate, DIR)
    (hlplan, traj) = interfaceLayer.run(state, world, pose)
    

    print "\n\n\n OUTPUT FROM INTERFACE LAYER\n\n"
    print hlplan
    print traj
w
