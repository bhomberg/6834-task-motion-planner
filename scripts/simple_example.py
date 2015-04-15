#!/usr/bin/env python

from mockPoseGenerator import *
from mockStateUpdate import *
from interface_layer import *

if __name__ == "__main__":
    state = None
    world = None
    pose = None

    interfaceLayer = InterfaceLayer('task_server_service', 'motion_server_service', MockPoseGenerator, mockStateUpdate)
    (hlplan, traj) = interfaceLayer.run(state, world, pose)
    
    print hlplan
    print traj
    
