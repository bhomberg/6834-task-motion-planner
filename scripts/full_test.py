#!/usr/bin/env python

from mockPoseGenerator import *
from mockMotionPlanner import *
from mockStateUpdate import *
from interface_layer import *
from plannerServer import *
from generateWorldMsg import *

if __name__ == '__main__':
    genWorld = generateWorldMsg()
    
    f = open('../states/one_cover','r')
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
    pose = None

    task_planner_server = TaskPlannerServer("~/indigo_ws/src/6834-task-motion-planner/FF-v2.3/", "~/indigo_ws/src/6834-task-motion-planner/domain")
    motion_planner_server = MockMotionPlannerServer(17)
    interfaceLayer = InterfaceLayer('task_server_service', 'motion_server_service', MockPoseGenerator, mockStateUpdate, '~/indigo_ws/src/6834-task-motion-planner/')
    (hlplan, traj) = interfaceLayer.run(state, world, pose)
    
    print hlplan
    print traj
