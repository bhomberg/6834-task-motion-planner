#!/usr/bin/env python

import sys
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *

MAX_TRAJ_COUNT = 999
task_server = None
motion_server = None
# TODO: if necessary, add in random seed for pose generators later

class InterfaceLayer(object):
    def __init__(self, ):
        pass
        
    def run_interface_layer(state, world, initialPose):
        # set up initial variables
        initial_state = state
        initial_world = world
        step = None
        hlplan = None
        partialTraj = None
        pose1 = None
        # create our tryRefine object, since that function needs to maintain its local variables
        sub = tryRefineClass()
        
        if hlplan == None: # ie, we haven't started planning yet
            hlplan = callTaskPlanner(state) # find a high level task plan
            step = 1
            partialTraj = None
            pose1 = initialPose
        # now that we have a high level plan, we try to actually turn it into motions in the real world
        while(resource limit not reached):
            # try to refine our plan, allowing for no errors
            (success, refinement) = sub.try_refine(pose1, state, world, hlplan, step, partialTraj, mode='errorFree')
            if success: # if it worked, we're done!
                return refinement
            trajCount = 0
            # if it didn't work, we need to start piece by piece
            while not success and trajCount < MAX_TRAJ_COUNT:
                # find a partial trajectory -- how far along the high level task plan can we find motion plans for?
                (partialTraj, pose2, failStep, failCause, state, world) = sub.try_refine(pose1, state, world hlplan, step, partialTraj, mode='partialTraj')
                # when we eventually failed, we failed because some object(s) were in the way -- we need to update our new task planning problem to incorporate that
                (state, world) = stateUpdate(state, world, failCause, failStep)
                # now, call the task planner again on the new state
                (success, newPlan) = callTaskPlanner(state)
                if success: # it may not be possible to find a new plan; if it is, update our high level plan 
                    # if it wasn't possible to find a new plan, we'll start over to try and refine, but we'll pick different things because of the randomization (I think -- we should double check this)
                    hlplan = hlplan[0:failStep] + newPlan
                    pose1 = pose2
                    step = failStep
                trajCount++
        # if we spend a lot of time with no success, give up and start over
        if trajCount == MAX_TRAJ_COUNT:
            state = initial_state
            step = 1
            partialTraj = None
            pose1 = initialPose


    def callTaskPlanner(state, world):
        # plan is an array of tuples where the first thing is the action and the rest are objects it acts on
        # state is organized as follows:
        #    list of objects in the world
        #    list of predicates true in the initial state
        #    list of predicates for the goal
        msg = task_domain()
        #output state to file
	f = open('state', 'w')
        f.write('(define (problemtask)')
        f.write('(:domain taskmotion)')
        f.write('(:objects ')
        for o in state[0]:
            f.write(o + ' ')
        f.write(')')
        f.write('(:init ')
        for i in state[1]:
            f.write('(')
            for a in i:
                f.write(a + ' ')
            f.write(')\n')
        f.write(')\n')
        f.write('(:goal ')
        for g in state[2]:
            f.write('(')
            for a in g:
                f.write(a + ' ')
            f.write(')\n')
        f.write(')')
        f.close()
        msg.task_file = 'state'
        resp = task_server(msg)
        # parse plan file into appropriate action tuple
        plan = []
	l = resp.plan.plan.split('\n')
	plan = [tuple(line.split(' ')) for line in l]

        return (resp.plan.error, plan)

    def stateUpdate(state, world, failCause, failStep):
        return state

    def get_motion_plan(world, action, goals):
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

    def MPErrs(pose1, pose2, state, world, action):
        obstacles = world.world.collision_objects
	a_whole_new_world = World()
	a_whole_new_world.robot = world.robot
	a_whole_new_world.world = world.world
	
	for i in range(len(obstacles)):
            if i == 0:
                (~, ~, success) = get_motion_plan(world, action, pose2)
                if success:
                    return []
            else:
                for l in itertools.combinations(range(len(obstacles)), i):
                    updated_obstacles = obstacles
                    for item in l:
                        updated_obstacles.pop(item)
                    a_whole_new_world.world.collision_objects = updated_obstacles
                    (~, ~, success) = get_motion_plan(a_whole_new_world, action, pose2)
                    if success:
                        return [obstacles[i].id for i in l]
        print "ERROR!"


class tryRefineClass:
    def __init__(self):
        self.world = None
        self.state = None
        self.index = None
        self.traj = None
        self.called = False
        self.old_hlplan = None
        self.pose1 = None
        self.axn = None
        self.nextaxn = None
        self.poseGen = PoseGenerator()

    def try_refine(initialPose, state, world, hlplan, step, trajprefix, mode='errorFree'):
        # if this is the first time it's been called or if there's a new high level plan, update variables!
        if self.called == False or self.old_hlplan != hlplan:
            self.index = step - 1
            self.state = state
            self.traj = trajprefix
            self.pose1 = initialPose
            self.called = True
        # progressively try and find a motion plan for each action as we go through the plan
        while step-1 <= self.index and self.index <= len(hlplan):
            # start by figuring out our actions and poses, use the pose generators to do that
            self.axn = hlplan[index]
            self.nextaxn = hlplan[index+1]
            self.pose2 = self.poseGen(self.nextaxn).next() # TODO: make this match what Veronica's pose generators actually do, make sure her pose generators return none if not defined?
            if self.pose2 == None: # pose 2 is not defined
                self.poseGen(self.nextaxn).reset() #TODO: make veronica's pose generators match this
                self.pose1 = self.poseGen(self.axn).next()
                index--
                self.traj = traj.delSuffixFor(self.axn) # TODO: define this (trajectory object?)
            else:
                # try and find a motion plan!
                (succeeds, state, motionPlan) = get_motion_plan(self.pose1, self.pose2)
                if succeeds: # if it succeeds, yay, we're done, keep going
                    if self.index == len(hlplan)+1:
                        return traj
                    self.traj = self.traj.append(motionPlan)
                    self.index++
                    self.pose1 = self.pose2
                else if mode == 'partialTraj':
                    # if it fails, then we need to find out why it failed -- what's blocking?
                    # I feel like we need to actually do the backtracking search on all of the poses before deciding that we failed, but maybe I'm missing something?  we should look carefully through this tomorrow
                    return (self.pose1, self.traj, self.index+1, MPErrs(self.pose1, self.pose2, self.state, self.world), self.state, self.world)

if __name__ == "__main__":
    rospy.wait_for_service('task_server_service')
    task_server = rospy.ServiceProxy('task_server_service', task_service)
    rospy.wait_for_service('motion_server_service')
    motion_server = rospy.ServiceProxy('motion_server_service', motion_service)
    state = None # TODO: acquire state message and put in proper format
    initialPose = None # TODO: acquire initial pose and put in proper format 
    world = None #TODO: acquire world state 
    run_interface_layer(state, initialPose, world)
 
