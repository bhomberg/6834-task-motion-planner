#!/usr/bin/env python

import sys
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *

MAX_TRAJ_COUNT = 999
# TODO: if necessary, add in random seed for pose generators later

def task_planner_client():
    rospy.wait_for_service('task_server_service')
    task_server = rospy.ServiceProxy('task_server_service', task_service)
    msg = task_domain()
    msg.task_file = 'problem0'
    resp1 = task_server(msg)
    return resp1.plan.error, resp1.plan.file

def run_interface_layer(state, initialPose):
    initial_state = state
    step = None
    hlplan = None
    partialTraj = None
    pose1 = None
    sub = tryRefineClass()
    if hlplan == None:
        hlplan = callTaskPlanner(state)
        step = 1
        partialTraj = None
        pose1 = initialPose
    while(resource limit not reached):
        (success, refinement) = sub.try_refine(pose1, hlplan, step, partialTraj, mode='errorFree')
        if success:
            return refinement
        success = False
        trajCount = 0
        while not success and trajCount < MAX_TRAJ_COUNT:
            (partialTraj, pose2, failStep, failCause) = sub.try_refine(pose1, hlplan, step, partialTraj, mode='partialTraj')
            state = stateUpdate(state, failCause, failStep)
            (success, newPlan) = callTaskPlanner(state)
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


def callTaskPlanner(state):
    # plan is an array of tuples, let's say, where the first thing is the action and the rest are objects it acts on
    return plan

def stateUpdate(state, failCause, failStep):
    return state

def get_motion_plan(pose1, pose2):
    pass

def MPErrs(pose1, pose2):
    pass


class tryRefineClass:

    def __init__(self):
        self.index = None
        self.traj = None
        self.called = False
        self.old_hlplan = None
        self.pose1 = None
        self.axn = None
        self.nextaxn = None
        self.poseGen = PoseGenerator()

    def try_refine(initialPose, hlplan, step, trajprefix, mode='errorFree'):
        if self.called == False or self.old_hlplan != hlplan:
            self.index = step - 1
            self.traj = trajprefix
            self.pose1 = initialPose
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
                (succeeds, motionPlan) = get_motion_plan(self.pose1, self.pose2)
                if succeeds:
                    if self.index == len(hlplan)+1:
                        return traj
                    self.traj = self.traj.append(motionPlan)
                    self.index++
                    self.pose1 = self.pose2
                else if mode == 'partialTraj':
                    return (self.pose1, self.traj, self.index+1, MPErrs(self.pose1, self.pose2))

if __name__ == "__main__":
    run_interface_layer()
 
