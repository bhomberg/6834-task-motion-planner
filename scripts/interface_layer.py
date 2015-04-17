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
task_server = None
motion_server = None
# TODO: if necessary, add in random seed for pose generators later
DIR = '/home/vmlane/catkin_ws/src/6834-task-motion-planner/state'

class InterfaceLayer(object):
    # This class is NOT a complete class.  Several functions must be implemented in order
    # to work with your specific motion planner and your specific task planner.  In particular, 
    # the callTaskPlanner function must be implemented to take inputs in the appropriate format
    # and return a high level plan in the correct format.  Similarly, get_motion_plan must be
    # implemented to do the same with the motion planner.  Finally, stateUpdate must be 
    # implemented to correctly handle updating the state based on errors identified via the
    # interface layer.  The class SpecificInterfaceLayer in this file properly extends
    # InterfaceLayer and implements the appropriate methods.
    
    def __init__(self, ):
        pass
        
    def run_interface_layer(self, state, initialPose, world):
        # set up initial variables
        initial_state = state
        initial_world = world
        step = None
        hlplan = None
        partialTraj = []
        pose1 = None
        num_iters = 0
        # create our tryRefine object, since that function needs to maintain its local variables
        sub = tryRefineClass()
        print state
        print world
        
        if hlplan == None: # ie, we haven't started planning yet
            (error, hlplan) = self.callTaskPlanner(state) # find a high level task plan
            step = 0
            partialTraj = []
            pose1 = initialPose
            if error:
                return "Error! Cannot find any high level plan. :("
            print "HLPlan: ", hlplan
            # now that we have a high level plan, we try to actually turn it into motions in the real world
        while(num_iters < MAX_ITERS):
            print "HIGHEST LEVEL ALG 1 ITERATION: ", num_iters
            num_iters+=1
            # try to refine our plan, allowing for no errors
            (success, t6, traj, t1, t2, t3, t4) = sub.try_refine(pose1, state, world, hlplan, step, partialTraj, 'errorFree')
            #print "blaH
            print traj
            if success: # if it worked, we're done!
                return traj
            trajCount = 0
            # if it didn't work, we need to start piece by piece
            while not success and trajCount < MAX_TRAJ_COUNT:
                print "INNER LEVEL ALG 1 ITERATION: ", trajCount
                #print "enter while"
                # find a partial trajectory -- how far along the high level task plan can we find motion plans for?
                (success, pose2, partialTraj, failStep, failCause, state, world) = sub.try_refine(pose1, state, world, hlplan, step, partialTraj, mode='partialTraj')
                #print "bloop"
                print "Partial traj, step: ", failStep, ", failcause: ", failCause, ", traj: ", partialTraj
                if success: # if we succeeded, return!
                    return partialTraj
                # when we eventually failed, we failed because some object(s) were in the way -- we need to update our new task planning problem to incorporate that
                (state) = self.stateUpdate(state, failCause, failStep, hlplan, world)
                # now, call the task planner again on the new state
                (error, newPlan) = self.callTaskPlanner(state)
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
                state = initial_state
                trajCount = 0
                step = 1
                partialTraj = []
                pose1 = initialPose

    def MPErrs(self, pose1, pose2, state, world, action):
        obstacles = world.world.collision_objects
	a_whole_new_world = World()
	a_whole_new_world.robot = world.robot
	a_whole_new_world.world = world.world
	
	for i in range(len(obstacles)):
            if i == 0:
                (a, b, success) = get_motion_plan(world, action, pose2)
                if success:
                    return []
            else:
                for l in itertools.combinations(range(len(obstacles)), i):
                    updated_obstacles = obstacles
                    for item in l:
                        updated_obstacles.pop(item)
                    a_whole_new_world.world.collision_objects = updated_obstacles
                    (a, b, success) = get_motion_plan(a_whole_new_world, action, pose2)
                    if success:
                        return [obstacles[i].id for i in l]
        print "ERROR IN MP ERRORS!"

    def callTaskPlanner(self, state):
        # want to return: (success, state)
        pass

    def stateUpdate(self, state, failCause, failStep, hlplan, world):
        # want to return: state
        pass

    def get_motion_plan(self, world, action, goals):
        # want to return: (world, motion plan, success)
        pass

class SpecificInterfaceLayer(InterfaceLayer):

    def callTaskPlanner(self, state):
        # plan is an array of tuples where the first thing is the action and the rest are objects it acts on
        # state is organized as follows:
        #    list of objects in the world
        #    list of predicates true in the initial state
        #    list of predicates for the goal
        msg = task_domain()
        #output state to file
	f = open(DIR, 'w')
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
        msg.task_file = DIR
        resp = task_server(msg)
        # parse plan file into appropriate action tuple
        plan = []
	l = resp.plan.plan.split('\n')
	plan = [tuple(line.split(' ')) for line in l]
        plan[-1] = ('BUFFER DEFAULT ACTION', 'BLAH', 'BLAH', 'BLAH', 'BLAH')
        #print "PLAN: ", plan
        return (resp.plan.error, plan)

    def stateUpdate(self, state, failCause, failStep, hlplan, world):
        # stateUpdate will need to be updated based on the specific problem
        # currently set up for mocks
        for i in range(failStep):
            action = hlplan[i]
            if action[0] == 'PICKUP':
                state[1].remove( ('EMPTY', action[2]) )
                state[1].append( ('NOT', 'EMPTY', action[2]) )
                state[1].remove( ('ROBOTAT', action[3]) )
                state[1].append( ('ROBOTAT', action[4]) )
                state[1].append( ('NOT', 'ROBOTAT', action[3]) )
                state[1].append( ('IN', action[1], action[2]) )
                for obj in world.keys():
                    state[1].append( ('NOT', 'OBSTRUCTS', action[1], obj) )
                    state[1].append( ('NOT', 'PDOBSTRUCTS', action[1], obj, 'S') ) # MOCK
                    t = ('OBSTRUCTS', action[1], obj)
                    if t in state[1]:
                        state[1].remove(t)
                    t = ('PDOBSTRUCTS', action[1], obj, 'S') # MOCK
                    if t in state[1]:
                        state[1].remove(t)
                for loc in world.locations(): # TODO: fix this based on Alex's structure
                    t = ('AT', action[1], loc)
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


        action = hlplan[failStep]
        if action[0] == 'PICKUP':
            obj_to_pickup = action[1]
            for obj in failCause:
                state[1].append( ('OBSTRUCTS', obj, obj_to_pickup))
                print "ADDED STATE (pickup): ", state[1][-1]
                print state[1]
        elif action[0] == 'PUTDOWN':
            obj_to_putdown = action[1]
            tloc = action[5]
            for obj in failCause:
                state[1].append( ('PDOBSTRUCTS', obj, obj_to_putdown, tloc))
                print "ADDED STATE (putdown): ", state[1][-1]
                print state[1]
        return state

    def get_motion_plan(self, world, action, goals):
        return mocked_motion_plan(world, action, goals)

        try:
            msg = motion_plan_parameters()
            msg.state = world
            msg.action = action
            msg.goals = goals
        
            resp = motion_server(msg)
        
            return (resp.plan.state, res.plan.motion, resp.plan.success)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return (world, [], False)

    def mocked_motion_plan(self, world, action, goals):
        axn = action[0]
        if axn == putdown:
            obj = action[1]
            loc = action[5]
            world[obj] = loc
            return (world, action, True)
            
        if axn == pickup:
            grip = action[2]
            obj = action[1]
            if obj == 'block1' and world['block1'] == 'I' and world['block2'] == 'I':
                return (world, [], False)
            world[obj] = grip
            return (world, action, True)
            

class tryRefineClass:
    def __init__(self):
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
        self.poseGen = MockPoseGenerator() # TODO: REMOVE THIS
        
    def try_refine(self, initialPose, state, world, hlplan, step, trajprefix, mode):
        # if this is the first time it's been called or if there's a new high level plan, update variables!
        if self.called == False or self.old_hlplan != hlplan:
            print "TRY REFINE: FIRST TIME BEING CALLED OR NEW HLPLAN"
            self.index = step
            self.state = state
            self.traj = trajprefix
            self.pose1 = initialPose
            self.called = True
            self.old_hlplan = hlplan
            self.hlplan = hlplan
            self.poseGen.resetAll()
            self.world = world
        # progressively try and find a motion plan for each action as we go through the plan
        while step <= self.index and self.index < len(hlplan) - 1: #NOTE: changed this from <= to < because of out of bounds error
            print "TRY REFINE ITERATING LOOP: ", self.index
            # start by figuring out our actions and poses, use the pose generators to do that
            self.axn = hlplan[self.index]
            self.nextaxn = hlplan[self.index+1]
            self.pose2 = self.poseGen.next(self.nextaxn)
            if self.pose2 == None: # pose 2 is already defined, so we should backtrack if in Error free mode, otherwise return that we've failed
                if mode == 'partialTraj': # we failed -- don't bother backtracking since we're just looking for a partial trajectory, so let's figure out what's blocking it -- BIANCA's MODIFICATION
                    return (False, self.pose1, self.traj, self.index, self.mocked_MPErrs(self.pose1, self.pose2, self.state, self.world, self.axn), self.state, self.world)
                self.poseGen.reset(self.nextaxn)
                self.pose1 = self.poseGen.next(self.axn)
                self.index-=1
                self.traj = self.traj[0:-1] #cut off the motion plan corresponding to that action
                #self.traj = traj.delSuffixFor(self.axn)
            else:
                # try and find a motion plan!
                #print 'world: ', self.world
                #print 'axn: ', self.axn
                #print 'pose2: ', self.pose2
                (world, motionPlan, succeeds) = self.mocked_motion_plan(self.world, self.axn, self.pose2) # TODO: fix how we access the motion plan
                if succeeds: # if it succeeds, either we're done or we can keep going keep going
                    print "CHECK: ", self.index, len(hlplan) - 1
                    if self.index == len(hlplan) - 2:
                        return (True, self.pose1, self.traj, self.index+1, [], self.state, self.world)
                    self.traj.append(motionPlan)
                    self.index+=1
                    self.pose1 = self.pose2
                #if mode == 'partialTraj':
                #    # if it fails, then we need to find out why it failed -- what's blocking?
                #    return (self.pose1, self.traj, self.index+1, MPErrs(self.pose1, self.pose2, self.state, self.world), self.state, self.world)
        # we finished!
        print "FINISHED ITERATING, ABOUT TO RETURN"
        print self.index
        self.index += 1
        #print (False, self.pose1, self.traj, self.index+1, [], self.state, self.world)
        return (False, self.pose1, self.traj, self.index, [], self.state, self.world)


    def mocked_motion_plan(self, world, action, goals):
        axn = action[0]
        if axn == 'PUTDOWN':
            obj = action[1]
            loc = action[5]
            world[obj] = loc
            return (world, action, True)
            
        if axn == 'PICKUP':
            grip = action[2]
            obj = action[1]
            #print "object: ", obj
            if 'BLOCK1' in world and 'BLOCK2' in world:
                if obj == 'BLOCK1' and world['BLOCK1'] == 'I' and world['BLOCK2'] == 'I':
                    return (world, [], False)
            world[obj] = grip
            return (world, action, True)

    def mocked_MPErrs(self, pose1, pose2, state, world, action):
        obstacles = world.keys()
        #print obstacles
	a_whole_new_world = copy.deepcopy(world)
	#a_whole_new_world.robot = world.robot
	#a_whole_new_world.world = world.world
	
	for i in range(len(obstacles)):
            if i == 0:
                (a, b, success) = self.mocked_motion_plan(world, action, pose2)
                if success:
                    return []
            else:
                for l in itertools.combinations(range(len(obstacles)), i):
                    #print l
                    a_whole_new_world = copy.deepcopy(world)
                    #updated_obstacles = obstacles
                    for item in l:
                        #print "ITEM: ", obstacles[item]
                        a_whole_new_world.pop(obstacles[item])
                    #a_whole_new_world.world.collision_objects = updated_obstacles
                    (a, b, success) = self.mocked_motion_plan(a_whole_new_world, action, pose2)
                    if success:
                        return [obstacles[i] for i in l]
        print "ERROR!"

class MockPoseGenerator:
    def __init__(self):
        self.state = dict()

    def next(self, action):
        if action in self.state:
            # if there are letters left, return it
            if len(self.state[action]) > 0:
                return self.state[action].pop()
            else:
                return None
        else:
            # generate a random order & return a letter
            self.state[action] = ['A','B','C','D','E','F','G','H','I','J']
            shuffle(self.state[action])
            return self.state[action].pop()

    def reset(self, action):
        if action in self.state:
            self.state[action] = []
    def resetAll(self):
        self.state = dict()
            
if __name__ == "__main__":
    rospy.init_node('interface_node')
    print "initialized node"
    rospy.wait_for_service('task_server_service')
    task_server = rospy.ServiceProxy('task_server_service', task_service)
    #rospy.wait_for_service('motion_server_service')
    #motion_server = rospy.ServiceProxy('motion_server_service', motion_service)
    # ASSUME WE ARE GIVEN THE INPUT TO PARSE AS A STRING CALLED init_state_string
    state = [[]]*3
    l = init_state_string.split('\n')
    state[0] = l[0].split(',')
    k = l[1].split(',')
    state[1] = [i.split(' ') for i in k]
    k = l[2].split(',')
    state[2] = [i.split(' ') for i in k]
    initialPose = l[3]
    #state.append(['BLOCK1 - physob', 'BLOCK2 - physob', 'LEFTARM - gripper', 'GP_BLOCK1 - pose', 
    #            'GP_BLOCK2 - pose', 'PDP_BLOCK1_S - pose', 'PDP_BLOCK2_S - pose', 'INITPOSE - pose',
    #            'S - location'])
    #state.append([('ROBOTAT', 'INITPOSE'), ('EMPTY', 'LEFTARM'), ('ISGPFG', 'GP_BLOCK1', 'BLOCK1'),
    #            ('ISGPFG', 'GP_BLOCK2', 'BLOCK2'), ('ISGPFPD', 'PDP_BLOCK1_S', 'BLOCK1', 'S'),
    #            ('ISGPFPD', 'PDP_BLOCK2_S', 'BLOCK2', 'S'), ('ISLFPD', 'S', 'BLOCK1'),
    #            ('ISLFPD', 'S', 'BLOCK2')])
    #state.append([('AT', 'BLOCK1', 'S')])
    #initialPose = 'INITPOSE'
    world = {'BLOCK1': 'I', 'BLOCK2': 'I'}
    interface = SpecificInterfaceLayer()
    print "about to send to go"
    blah =  interface.run_interface_layer(state, initialPose, world)
    print "FINAL ANSWER: ", blah
    print "END"
    rospy.spin()
 
