#!/usr/bin/env python

import time
import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../src/'))
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *
import copy
import itertools
from random import shuffle
from mockPoseGenerator import *
from mockMotionPlanner import *
from mockStateUpdate import *
from interface_layer_organized import *
from planner_server import *
from generateWorldMsg import *
from numObstructs import *
import re
from stateGenerator import *
from generateWorldMsg import *

# rosrun mock motion planner srv
# rosrun mock task planner srv

numTests = 10

genWorld = generateWorldMsg()
genState = StateGenerator()

DIR_6834 = '/home/ragtz/indigo_workspace/src/6834-task-motion-planner/'

# One Cover Bechmark
out_file = open('benchmark.output','w')
world_shapes = ["SQUARE","CROSS","X","HLINE","VLINE"]
# world_shapes = ["CROSS","X","HLINE","VLINE"]
world_sizes = xrange(3,14,2)

for shape in world_shapes:
	for size in world_sizes:
		# set up the world
		world = genWorld.generateWorld(shape,size)
		filename = DIR_6834+"states/" + shape + "_" + str(size)
		state = genState.genStateFromWorld(world,filename)
		
		f = open(filename,'r')
		init_state_string = f.read()
		state = [[]]*3
		l = init_state_string.split('\n')
		state[0] = l[0].split(',')
		k = l[1].split(',')
		state[1] = [tuple(i.split(' ')) for i in k]
		k = l[2].split(',')
		state[2] = [tuple(i.split(' ')) for i in k]
		pose = l[3]

		# Difficulty Metric
		numBlocks = len(world.world.movable_objects)
		numObs = numObstructsWorld(world, 17)

		numMotionPlannerCalls = []
		numTaskPlannerCalls = []
		runTimes = []

		# run multiple tests for this world
		for i in range(numTests):
			poseGen = MockPoseGenerator()
			interfaceLayer = InterfaceLayer('task_server_service', 'motion_server_service', poseGen, mockStateUpdate, DIR_6834)
			tic = time.time()
			(hlplan, traj) = interfaceLayer.run(state, world, pose)
			runTime = time.time() - tic
			numMotionPlannerCalls.append(interfaceLayer.numMotionPlannerCalls)
			numTaskPlannerCalls.append(interfaceLayer.numTaskPlannerCalls)
			runTimes.append(runTime)
			fname=DIR_6834+"outputResults/"+shape+str(size)+"_"+str(i)+".output"
			plan_traj = open(fname,'w')
			# save hlplan & trajectory to output folder
			plan_traj.write(str(hlplan)+"\n"+"\n")
			plan_traj.write(str(traj)+"\n"+"\n")

		# Averaging
		avgMotionPlannerCalls = sum(numMotionPlannerCalls)/len(numMotionPlannerCalls)
		avgTaskPlannerCalls = sum(numTaskPlannerCalls)/len(numTaskPlannerCalls)
		avgRunTimes = sum(runTimes)/len(runTimes)

		#print  TODO: print world description
		s = 'State File: ' + filename + "\n"
		s += 'Shape: ' + shape +"\n"
		s += 'Size: ' + str(size) + "\n"
		s += 'Number of Blocks: ' + str(numBlocks) + "\n"
		s += 'Number of Obstructions: ' + str(numObs) + "\n"
		s += 'Run Time: ' + str(avgRunTimes) + "\n"
		s += 'Task Planner Calls: ' + str(avgTaskPlannerCalls) + "\n"
		s += 'Motion Planner Calls: ' + str(avgMotionPlannerCalls) + "\n"
		s += "\n\n"
		print s
		out_file.write(s)
		# TODO save to file
out_file.close()
