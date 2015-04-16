#!/usr/bin/env python

import sys
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *
import copy
import itertools
from random import shuffle
from mockPoseGenerator import *
from mockMotionPlanner import *
from mockStateUpdate import *
from interface_layer_organized import *
from planner_server import *
from generateWorldMsg import *

MAX_TRAJ_COUNT = 5
MAX_ITERS = 100

class InterfaceLayer(object):    
    def __init__(self, taskServerName, motionServerName, poseGenerator, stateUpdate, directory):        
        self.taskServerName = taskServerName
        self.taskServer = None
        
        self.motionServerName = motionServerName
        self.motionServer = None
        
        self.poseGenerator = poseGenerator
        self.stateUpdate = stateUpdate
        
        self.directory = directory
        print "Initialized interface layer!"
        
    def _callTaskPlanner(self, state):
        # plan is an array of tuples where the first thing is the action and the rest are objects it acts on
        # state is organized as follows:
        #    list of objects in the world
        #    list of predicates true in the initial state
        #    list of predicates for the goal
        msg = task_domain()
        #output state to file
        f = open(self.directory + 'state', 'w')
        f.write('(define (problem problemtask)\n')
        f.write('(:domain taskmotion)\n')
        f.write('(:objects ')
        for o in state[0]:
            f.write(o + ' ')
        f.write(')\n')
        f.write('(:init ')
        #print "state[1]: ", state[1]
        #print "first one: ", state[1][1]
        for i in state[1]:
            f.write('(')
            for a in i:
                f.write(a + ' ')
                if a == 'NOT':
                    f.write('(')
            if i[0] == 'NOT':
                f.write(')')
            f.write(')\n')
        f.write(')\n')
        f.write('(:goal ')
        for g in state[2]:
            f.write('(')
            for a in g:
                f.write(a + ' ')
                if a == 'NOT':
                    f.write('(')
            if g[0] == 'NOT':
                f.write(')')
            f.write(')\n')
        f.write(')\n)')
        f.close()
        msg.task_file = self.directory + 'state'
        resp = self.taskServer(msg)
        # parse plan file into appropriate action tuple
        plan = []
        l = resp.plan.plan.split('\n')
        plan = [tuple(line.split(' ')) for line in l]
        #plan.insert(0, ('BUFFER DEFAULT ACTION', 'BLAH', 'BLAH', 'BLAH', 'BLAH'))
        plan = plan[0:-1]
        #print "PLAN: ", plan
        return (resp.plan.error, plan)
        
    def _callMotionPlanner(self, world, action, goals):
        # return: (world, motion plan, success)
        try:
            msg = motion_plan_parameters()
            msg.state = world

            str_action = '('
            for elem in action:
                str_action += elem + ','
            msg.action = str_action[:-1] + ')'

            msg.goals = goals
        
            res = self.motionServer(msg)
        
            return (res.plan.state, res.plan.motion, res.plan.success)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return (world, [], False)
       
    # TODO: Ensure that this works for message types     
    def _mpErrs(self, pose1, pose2, state, world, action):
        obstacles = copy.deepcopy(world.world.movable_objects)
        a_whole_new_world = world_state()
        a_whole_new_world.robot = copy.deepcopy(world.robot)
        a_whole_new_world.world = copy.deepcopy(world.world)
	
        for i in range(len(obstacles)):
            if i == 0:
                print "Pose: ", pose2
                (a, b, success) = self._callMotionPlanner(world, action, pose2)
                if success:
                    return []
            else:
                for l_tuple in itertools.combinations(range(len(obstacles)), i):
                    updated_obstacles = copy.deepcopy(obstacles) #double check that this is a deep copy
                    l = list(l_tuple)
                    l.reverse()
                    #print "index of things to pop: ", l
                    #print "initial list of obstacles: ", [obstacle.id for obstacle in obstacles]
                    for item in l:
                        updated_obstacles.pop(item)
                    #print "updated: ", [obstacle.id for obstacle in updated_obstacles]
                    if action[1] in [obstacle.id for obstacle in updated_obstacles]:
                        a_whole_new_world.world.movable_objects = updated_obstacles
                        (a, b, success) = self._callMotionPlanner(a_whole_new_world, action, pose2)
                        if success:
                            return [obstacles[i].id for i in l]
        print "ERROR IN MP ERRORS!"
        
    def run(self, state, world, pose):
        
        # start task server client
        rospy.wait_for_service(self.taskServerName)
        self.taskServer = rospy.ServiceProxy(self.taskServerName, task_service)
        
        # start motion server client
        rospy.wait_for_service(self.motionServerName)
        self.motionServer = rospy.ServiceProxy(self.motionServerName, motion_service)

        # set up initial variables
        initState = state
        initWorld = world
        initPose = pose
        step = None
        hlplan = None
        partialTraj = []
        pose1 = None
        num_iters = 0
        prev_fail_step = 0
        # create our tryRefine object, since that function needs to maintain its local variables
        trajRefiner = TryRefine(self)
        
        if hlplan == None: # ie, we haven't started planning yet
            (error, hlplan) = self._callTaskPlanner(state) # find a high level task plan
            step = 0
            partialTraj = []
            pose1 = initPose
            if error:
                return "Error! Cannot find any high level plan. :("
            hlplan.insert(0, (' ', ' ', ' ',' ', ' '))
            print "HLPlan: ", hlplan
            # now that we have a high level plan, we try to actually turn it into motions in the real world
            
        while(num_iters < MAX_ITERS):
            print "HIGHEST LEVEL ALG 1 ITERATION: ", num_iters
            num_iters+=1
            
            # try to refine our plan, allowing for no errors
            (success, t6, traj, t1, t2, t3, t4) = trajRefiner.tryRefine(pose1, state, world, hlplan, step, partialTraj, 'errorFree')
            print traj
            
            if success: # if it worked, we're done!
                return (hlplan, traj)
            trajCount = 0
            
            # if it didn't work, we need to start piece by piece
            while not success and trajCount < MAX_TRAJ_COUNT:
                print "INNER LEVEL ALG 1 ITERATION: ", trajCount

                # find a partial trajectory -- how far along the high level task plan can we find motion plans for?
                (success, pose2, partialTraj, failStep, failCause, state, world) = trajRefiner.tryRefine(pose1, state, world, hlplan, step, partialTraj, mode='partialTraj')
                print "Partial traj, step: ", failStep, ", failcause: ", failCause, ", traj: ", partialTraj
                
                if success: # if we succeeded, return!
                    return (hlplan, partialTraj)
                    
                # when we eventually failed, we failed because some object(s) were in the way -- we need to update our new task planning problem to incorporate that
                (state) = self.stateUpdate(state, failCause, failStep, prev_fail_step, hlplan, world)
                # now, call the task planner again on the new state
                (error, newPlan) = self._callTaskPlanner(state)
                
                if not error: # it may not be possible to find a new plan; if it is, update our high level plan 
                    # if it wasn't possible to find a new plan, we'll start over to try and refine, but we'll pick different things because of the randomization
                    hlplan = hlplan[0:failStep+1] + newPlan
                    pose1 = pose2
                    prev_fail_step = failStep
                    step = failStep
                    print "HLPlan: ", hlplan
                    
                trajCount+=1
                
            # if we spend a lot of time with no success, give up and start over
            if trajCount == MAX_TRAJ_COUNT:
                print "RESETING TRAJ COUNT"
                state = initState
                trajCount = 0
                step = 1
                partialTraj = []
                pose1 = initPose
                
        return (False, False)


class TryRefine(object):
    def __init__(self, interface):
        self.world = None
        self.state = None
        self.hlplan = None
        self.index = None
        self.actionNum = None
        self.traj = None
        self.called = False
        self.old_hlplan = None
        self.targetPose = [] # TODO: figure out exactly how we want to initialize this if this doesn't work
        self.axn = None
        self.nextaxn = None
        self.interface = interface
        
    def tryRefine(self, initPose, state, world, hlplan, step, trajprefix, mode):
        # if this is the first time it's been called or if there's a new high level plan, update variables!
        if self.called == False or self.old_hlplan != hlplan:
            print "TRY REFINE: FIRST TIME BEING CALLED OR NEW HLPLAN"
            self.index = step
            self.state = state
            self.traj = trajprefix
            self.pose1 = initPose
            self.called = True
            self.old_hlplan = hlplan
            self.hlplan = hlplan
            self.interface.poseGenerator.resetAll()
            self.world = world
            
        print "MODE: ", mode
        # progressively try and find a motion plan for each action as we go through the plan
        while step <= self.index and self.index < len(hlplan) - 1: #NOTE: changed this from <= to < because of out of bounds error
            print "TRY REFINE ITERATING LOOP: ", self.index
            # start by figuring out our actions and poses, use the pose generators to do that
            self.axn = hlplan[self.index]
            #if self.index+1 < len(hlplan):
            self.nextaxn = hlplan[self.index+1]
            #else:
            #    self.nextaxn = ('End')
            print "this action: ", self.axn
            print "next action: ", self.nextaxn
            print "index: ", self.index
            self.pose2 = self.interface.poseGenerator.next(self.nextaxn)
            
            if self.pose2 == None: # pose 2 is already defined, so we should backtrack if in Error free mode, otherwise return that we've failed
                #if mode == 'partialTraj': # we failed -- don't bother backtracking since we're just looking for a partial trajectory, so let's figure out what's blocking it -- BIANCA's MODIFICATION
                #    return (False, self.pose1, self.traj, self.index, self.interface._mpErrs(self.pose1, self.pose2, self.state, self.world, self.axn), self.state, self.world)
                self.interface.poseGenerator.reset(self.axn)
                self.pose1 = self.interface.poseGenerator.next(self.axn)
                self.index-=1
                self.traj = self.traj[0:-1] #cut off the motion plan corresponding to that action
                #self.traj = traj.delSuffixFor(self.axn)
            else:
                # try and find a motion plan!
                #print 'world: ', self.world
                #print 'nextaxn: ', self.nextaxn
                #print 'pose2: ', self.pose2
                (world, motionPlan, succeeds) = self.interface._callMotionPlanner(self.world, self.nextaxn, self.pose2) # TODO: fix how we access the motion plan
                if succeeds: # if it succeeds, either we're done or we can keep going keep going
                    print "CHECK: ", self.index, len(hlplan) - 1
                    if self.index == len(hlplan) - 1:
                        return (True, self.pose1, self.traj, self.index, [], self.state, self.world)
                    self.traj.append(motionPlan)
                    self.index+=1
                    self.pose1 = self.pose2
                    self.world = world
                elif mode == 'partialTraj':
                    # if it fails, then we need to find out why it failed -- what's blocking?
                    return (False, self.pose1, self.traj, self.index, self.interface._mpErrs(self.pose1, self.pose2, self.state, self.world, self.nextaxn), self.state, self.world)
                
        # we finished
        print "FINISHED ITERATING, ABOUT TO RETURN"
        print self.index
        self.index += 1
        self.interface.poseGenerator.resetAll()
        #print (False, self.pose1, self.traj, self.index+1, [], self.state, self.world)
        return (False, self.pose1, self.traj, self.index, [], self.state, self.world)

if __name__ == '__main__':
    rospy.init_node('interface_node')
    print "initialized node"

    genWorld = generateWorldMsg()
    
    f = open('/home/bhomberg/indigo_ws/src/6834-task-motion-planner/states/one_cover','r')
    init_state_string = f.read()
    
    state = [[]]*3
    l = init_state_string.split('\n')
    state[0] = l[0].split(',')
    k = l[1].split(',')
    state[1] = [tuple(i.split(' ')) for i in k]
    k = l[2].split(',')
    state[2] = [tuple(i.split(' ')) for i in k]
    pose = l[3]
    print state

    world = genWorld.generateWorld('SQUARE',3)

    poseGen = MockPoseGenerator()

    interfaceLayer = InterfaceLayer('task_server_service', 'motion_server_service', poseGen, mockStateUpdate, '/home/bhomberg/indigo_ws/src/6834-task-motion-planner/')
    (hlplan, traj) = interfaceLayer.run(state, world, pose)
    
    print "\n\n\n OUTPUT FROM INTERFACE LAYER\n\n"
    print hlplan
    print traj

    rospy.spin()
