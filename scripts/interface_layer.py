#!/usr/bin/env python

import sys
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from moveit_msgs.msg import *
from geometry_msgs.msg import *

MAX_TRAJ_COUNT = 999
task_server = None
# TODO: if necessary, add in random seed for pose generators later
motion_server = None

def run_interface_layer(state, world, initialPose):
    initial_state = state
    initial_world = world
    step = None
    hlplan = None
    partialTraj = None
    pose1 = None
    sub = tryRefineClass()
    if hlplan == None:
        hlplan = callTaskPlanner(state, world)
        step = 1
        partialTraj = None
        pose1 = initialPose
    while(resource limit not reached):
        (success, refinement) = sub.try_refine(pose1, state, world, hlplan, step, partialTraj, mode='errorFree')
        if success:
            return refinement
        success = False
        trajCount = 0
        while not success and trajCount < MAX_TRAJ_COUNT:
            (partialTraj, pose2, failStep, failCause, state, world) = sub.try_refine(pose1, state, world hlplan, step, partialTraj, mode='partialTraj')
            (state, world) = stateUpdate(state, world, failCause, failStep)
            (success, newPlan) = callTaskPlanner(state, world)
            if success:
                hlplan = hlplan[0:failStep] + newPlan
                pose1 = pose2
                step = failStep
            trajCount++
    if trajCount == MAX_TRAJ_COUNT:
        state = initial_state
        step = 1
        partialTraj = None
        pose1 = initialPose


def callTaskPlanner(state, world):
    # plan is an array of tuples, let's say, where the first thing is the action and the rest are objects it acts on
    msg = task_domain()
    #TODO: output state and world to file
    msg.task_file = 'state'
    resp = task_server(msg)
    # parse plan file into appropriate action tuple
    plan = []
    f = open(resp.plan.file)
    for line in f:
        plan.append(tuple(line.rsplit(' ')))
    return (resp.plan.error, plan)

def stateUpdate(state, world, failCause, failStep):
    return state

def get_motion_plan(world, state, action, goalPose):
    # TODO: Add timed try except???
    try:
        msg = motion_plan_parameters()
        msg.world = world
        msg.start = state
        msg.group_names = len(goalPose)*[actions[1]]
        
        # TODO: implement proper list of tuples management
        msg.goals = [goalPose]
        
        resp = motion_server(msg)
        
        return (resp.plan.success, res.plan.end_state, resp.plan.trajectory)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return (False, RobotState(), DisplayTrajectory())

def MPErrs(pose1, pose2, state, world):
    pass


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
        if self.called == False or self.old_hlplan != hlplan:
            self.index = step - 1
            self.state = state
            self.traj = trajprefix
            self.pose1 = initialPose
            self.called = True
        while step-1 <= self.index and self.index <= len(hlplan):
            self.axn = hlplan[index]
            self.nextaxn = hlplan[index+1]
            self.pose2 = self.poseGen(self.nextaxn).next() # TODO: make this match what Veronica's pose generators actually do, make sure her pose generators return none if not defined?
            if self.pose2 == None: # pose 2 is not defined
                self.poseGen(self.nextaxn).reset() #TODO: make veronica's pose generators match this
                self.pose1 = self.poseGen(self.axn).next()
                index--
                self.traj = traj.delSuffixFor(self.axn) # TODO: define this (trajectory object?)
            else:
                (succeeds, state, motionPlan) = get_motion_plan(self.pose1, self.pose2)
                if succeeds:
                    if self.index == len(hlplan)+1:
                        return traj
                    self.traj = self.traj.append(motionPlan)
                    self.index++
                    self.pose1 = self.pose2
                else if mode == 'partialTraj':
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
 
