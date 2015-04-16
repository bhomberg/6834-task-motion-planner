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

MAX_TRAJ_COUNT = 3
MAX_ITERS = 1

class InterfaceLayer(object):    
    def __init__(self, taskServerName, motionServerName, poseGenerator, stateUpdate):        
        self.taskServerName = taskServerName
        self.taskServer = None
        
        self.motionServerName = motionServerName
        self.motionServer = None
        
        self.poseGenerator = poseGenerator
        self.stateUpdate = stateUpdate
        
    def _callTaskPlanner(self, state):
        # plan is an array of tuples where the first thing is the action and the rest are objects it acts on
        # state is organized as follows:
        #    list of objects in the world
        #    list of predicates true in the initial state
        #    list of predicates for the goal
        msg = task_domain()
        #output state to file
	f = open('/home/bhomberg/indigo_ws/src/6834-task-motion-planner/state', 'w')
        f.write('(define (problem problemtask)\n')
        f.write('(:domain taskmotion)\n')
        f.write('(:objects ')
        for o in state[0]:
            f.write(o + ' ')
        f.write(')\n')
        f.write('(:init ')
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
        msg.task_file = '/home/bhomberg/indigo_ws/src/6834-task-motion-planner/state'
        resp = task_server(msg)
        # parse plan file into appropriate action tuple
        plan = []
	l = resp.plan.plan.split('\n')
	plan = [tuple(line.split(' ')) for line in l]
        plan[-1] = ('BUFFER DEFAULT ACTION', 'BLAH', 'BLAH', 'BLAH', 'BLAH')
        #print "PLAN: ", plan
        return (resp.plan.error, plan)
        
    def _callMotionPlanner(self, world, action, goals):
        # return: (world, motion plan, success)
        try:
            msg = motion_plan_parameters()
            msg.state = world
            msg.action = action
            msg.goals = goals
        
            res = self.motionServer(msg)
        
            return (res.plan.state, res.plan.motion, res.plan.success)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return (world, [], False)
       
    # TODO: Ensure that this works for message types     
    def _mpErrs(self, pose1, pose2, state, world, action):
        obstacles = world.world.collision_objects
	    a_whole_new_world = World()
	    a_whole_new_world.robot = world.robot
	    a_whole_new_world.world = world.world
	
	    for i in range(len(obstacles)):
            if i == 0:
                (a, b, success) = self._callMotionPlanner(world, action, pose2)
                if success:
                    return []
            else:
                for l in itertools.combinations(range(len(obstacles)), i):
                    updated_obstacles = obstacles
                    for item in l:
                        updated_obstacles.pop(item)
                    a_whole_new_world.world.collision_objects = updated_obstacles
                    (a, b, success) = self._callMotionPlanner(a_whole_new_world, action, pose2)
                    if success:
                        return [obstacles[i].id for i in l]
        print "ERROR IN MP ERRORS!"
        
    def run(self, state, world, pose):
        rospy.init_node('interface_node')
    
        # start task server client
        rospy.wait_for_service(self.taskServerName)
        self.taskServer = rospy.ServiceProxy(self.taskServerName, task_service)
        
        # start motion server client
        rospy.wait_for_service(self.motionServerName)
        self.motion_server = rospy.ServiceProxy(self.motionServerName, motion_service)
        
        # set up initial variables
        initState = state
        initWorld = world
        initPose = pose
        step = None
        hlplan = None
        partialTraj = []
        pose1 = None
        num_iters = 0
        # create our tryRefine object, since that function needs to maintain its local variables
        trajRefiner = TryRefine(self)
        
        print state
        print world
        
        if hlplan == None: # ie, we haven't started planning yet
            (error, hlplan) = self._callTaskPlanner(state) # find a high level task plan
            step = 0
            partialTraj = []
            pose1 = initPose
            if error:
                return "Error! Cannot find any high level plan. :("
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
                (state) = self.stateUpdate(state, failCause, failStep, hlplan, world)
                # now, call the task planner again on the new state
                (error, newPlan) = self._callTaskPlanner(state)
                
                if not error: # it may not be possible to find a new plan; if it is, update our high level plan 
                    # if it wasn't possible to find a new plan, we'll start over to try and refine, but we'll pick different things because of the randomization
                    hlplan = hlplan[0:failStep] + newPlan
                    pose1 = pose2
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
                
        rospy.spin()


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
            
        # progressively try and find a motion plan for each action as we go through the plan
        while step <= self.index and self.index < len(hlplan) - 1: #NOTE: changed this from <= to < because of out of bounds error
            print "TRY REFINE ITERATING LOOP: ", self.index
            # start by figuring out our actions and poses, use the pose generators to do that
            self.axn = hlplan[self.index]
            self.nextaxn = hlplan[self.index+1]
            self.pose2 = self.interface.poseGenerator.next(self.nextaxn)
            
            if self.pose2 == None: # pose 2 is already defined, so we should backtrack if in Error free mode, otherwise return that we've failed
                if mode == 'partialTraj': # we failed -- don't bother backtracking since we're just looking for a partial trajectory, so let's figure out what's blocking it -- BIANCA's MODIFICATION
                    return (False, self.pose1, self.traj, self.index, self.interface._mpErrs(self.pose1, self.pose2, self.state, self.world, self.axn), self.state, self.world)
                self.interface.poseGenerator.reset(self.nextaxn)
                self.pose1 = self.interface.poseGenerator.next(self.axn)
                self.index-=1
                self.traj = self.traj[0:-1] #cut off the motion plan corresponding to that action
                #self.traj = traj.delSuffixFor(self.axn)
            else:
                # try and find a motion plan!
                #print 'world: ', self.world
                #print 'axn: ', self.axn
                #print 'pose2: ', self.pose2
                (world, motionPlan, succeeds) = self.interface._callMotionPlanner(self.world, self.axn, self.pose2) # TODO: fix how we access the motion plan
                if succeeds: # if it succeeds, either we're done or we can keep going keep going
                    print "CHECK: ", self.index, len(hlplan) - 1
                    if self.index == len(hlplan) - 2:
                        return (True, self.pose1, self.traj, self.index+1, [], self.state, self.world)
                    self.traj.append(motionPlan)
                    self.index+=1
                    self.pose1 = self.pose2
                #if mode == 'partialTraj':
                #    # if it fails, then we need to find out why it failed -- what's blocking?
                #    return (self.pose1, self.traj, self.index+1, self.interface._mpErrs(self.pose1, self.pose2, self.state, self.world), self.state, self.world)
                
        # we finished!
        print "FINISHED ITERATING, ABOUT TO RETURN"
        print self.index
        self.index += 1
        #print (False, self.pose1, self.traj, self.index+1, [], self.state, self.world)
        return (False, self.pose1, self.traj, self.index, [], self.state, self.world)

