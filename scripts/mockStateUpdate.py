#!/usr/bin/env python

import sys
import rospy
from task_motion_planner.msg import *

# TODO: change world to message type not dictionary

def mockStateUpdate(state, failCause, failStep, prev_fail_step, hlplan, world):
        # stateUpdate will need to be updated based on the specific problem
        # currently set up for mocks
        print "prev fail step: ", prev_fail_step
        print "curr: ", failStep
        for i in range(failStep+1-prev_fail_step):
            print i+prev_fail_step
            action = hlplan[i+prev_fail_step]
            #print action
            #print state[1]
            if action[0] == 'PICKUP':
                state[1].remove( ('EMPTY', action[2]) )
                state[1].append( ('NOT', 'EMPTY', action[2]) )
                state[1].remove( ('ROBOTAT', action[3]) )
                state[1].append( ('ROBOTAT', action[4]) )
                state[1].append( ('NOT', 'ROBOTAT', action[3]) )
                state[1].append( ('IN', action[1], action[2]) )
                for obj in world.world.movable_objects:
                    state[1].append( ('NOT', 'OBSTRUCTS', action[1], obj.id) )                    
                    t = ('OBSTRUCTS', action[1], obj.id)
                    if t in state[1]:
                        state[1].remove(t)

                    for loc in world.world.surfaces:
                            state[1].append( ('NOT', 'PDOBSTRUCTS', action[1], obj.id, loc.id) )
                            t = ('PDOBSTRUCTS', action[1], obj.id, loc.id)
                            if t in state[1]:
                                    state[1].remove(t)
                for loc in world.world.surfaces: 
                    t = ('AT', action[1], loc.id)
                    if t in state[1]:
                        state[1].remove(t)
            if action[0] == 'PUTDOWN':
                state[1].remove( ('IN', action[1], action[2]) )
                state[1].append( ('NOT', 'IN', action[1], action[2]) )
                state[1].append( ('EMPTY', action[2]) )
                state[1].remove( ('NOT', 'EMPTY', action[2]) )
                state[1].append( ('AT', action[1], action[5]) )
                t = ('NOT', 'AT', action[1], action[5])
                if t in state[1]:
                    state[1].remove(t)
                state[1].remove( ('ROBOTAT', action[3]) )
                state[1].append( ('ROBOTAT', action[4]) )


        action = hlplan[failStep+1]
        if action[0] == 'PICKUP':
            obj_to_pickup = action[1]
            for obj in failCause:
                state[1].append( ('OBSTRUCTS', obj, obj_to_pickup))
                #print "ADDED STATE (pickup): ", state[1][-1]
                #print state[1]
        elif action[0] == 'PUTDOWN':
            obj_to_putdown = action[1]
            tloc = action[5]
            for obj in failCause:
                state[1].append( ('PDOBSTRUCTS', obj, obj_to_putdown, tloc))
                #print "ADDED STATE (putdown): ", state[1][-1]
                #print state[1]
        return state
