#!/usr/bin/env python

import time
import sys
import os
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
from stateGeneratorSmall import *
from generateSmallWorldMsg import *

# rosrun mock motion planner srv
# rosrun mock task planner srv

numTests = 10

genState =  StateGeneratorSmall()

DIR_6834 = os.path.abspath(os.path.dirname(__file__) + '/../') + '/'

# One Cover Bechmark
out_file = open(DIR_6834 + 'tests/smallTests.output','w')

# set up the world
worlds = generateWorlds()
genState.generateStates()

for filename,world in worlds.iteritems():
	print world
	
	filepath = DIR_6834+"states/" + filename		
	f = open(filepath,'r')
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

	fname=DIR_6834+"outputResults/"+filename+".output"
	outFile = open(fname,'w')
	# run multiple tests for this world
	for i in range(numTests):
		poseGen = MockPoseGenerator()
		interfaceLayer = InterfaceLayer('task_server_service', 'motion_server_service', poseGen, mockStateUpdate, DIR_6834)
		tic = time.time()
		ret = interfaceLayer.run(state, world, pose)
		runTime = time.time() - tic
		numMotionPlannerCalls.append(interfaceLayer.numMotionPlannerCalls)
		numTaskPlannerCalls.append(interfaceLayer.numTaskPlannerCalls)
		runTimes.append(runTime)
		outFile.write(str(ret)+"\n\n")
	outFile.close()

	# Averaging
	avgMotionPlannerCalls = sum(numMotionPlannerCalls)/len(numMotionPlannerCalls)
	avgTaskPlannerCalls = sum(numTaskPlannerCalls)/len(numTaskPlannerCalls)
	avgRunTimes = sum(runTimes)/len(runTimes)

	s = 'State File: ' + filename + "\n"
	s += 'Number of Surfaces: ' + str(len(world.world.surfaces))  + "\n"
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
