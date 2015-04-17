#!/usr/bin/python

import roslib
from task_motion_planner.msg import *

class GenerateSmallWorldMsg:
	def __init__(self):
		self.WORLD_GRID_SIZE = 17
		self.CENTER = self.WORLD_GRID_SIZE/2
		self.block_idx = 0

	# surface_and_blocks = a dictionary of surfaces and the number of blocks on each one
	def generateWorld(self,surface_and_blocks):
		my_world = world_state()
		my_world.world = world_obj()
		for s,numBlocks in surface_and_blocks.iteritems():
			# add the surface to the world
			surface = obj()
			surface.id = s
			my_world.world.surfaces.append(surface)
			# add the blocks
			self.generateBlockVLine(my_world.world,s,numBlocks)
			if not numBlocks == 0:
				self.generateWalls(my_world.world,s,-1,numBlocks)
		return my_world

	def generateBlockVLine(self,my_world,surface_id,length):
		for i in xrange(self.CENTER,self.CENTER+length):
			self._add_block(my_world,surface_id,i)

	def generateWalls(self,my_world,surface_id,lb,ub):
		for i in xrange(lb,ub):
			# left wall
			o = obj()
			o.loc.surface_id = surface_id
			o.loc.x = self.CENTER - 1
			o.loc.y = self.CENTER + i
			my_world.walls.append(o)
			# right wall
			o2 = obj()
			o2.loc.surface_id = surface_id
			o2.loc.x = self.CENTER + 1
			o2.loc.y = self.CENTER + i
			my_world.walls.append(o2)
		# top wall
		o3 = obj()
		o3.loc.surface_id = surface_id
		o3.loc.x = self.CENTER
		o3.loc.y = self.CENTER + ub
		my_world.walls.append(o3)



	def _add_block(self,my_world,surface_id,j):
		o = obj()
		o.id = 'BLOCK' + str(self.block_idx)
		# green block
		self.block_idx += 1 
		o.loc.surface_id = surface_id
		o.loc.x = self.CENTER
		o.loc.y = j
		o.loc.grasped = False
		my_world.movable_objects.append(o)
def generateWorlds():
	g = GenerateSmallWorldMsg()
	worlds = dict()
	surfaces = dict()
	surfaces['I'] = 2
	surfaces['S'] = 1
	worlds['state3blocks_1'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 2
	surfaces['A'] = 1
	surfaces['S'] = 0
	worlds['state3blocks_2'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 3
	surfaces['A'] = 0
	surfaces['S'] = 0
	worlds['state3blocks_3'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 3
	surfaces['A'] = 0
	surfaces['S'] = 0
	worlds['state3blocks_4'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 3
	surfaces['S'] = 1
	worlds['state4blocks_1'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 2
	surfaces['S'] = 2
	worlds['state4blocks_2'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 4
	surfaces['A'] = 0
	surfaces['S'] = 0
	worlds['state4blocks_3'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 3
	surfaces['A'] = 1
	surfaces['S'] = 0
	worlds['state4blocks_4'] = g.generateWorld(surfaces)
	return worlds
if __name__ == "__main__":
	g = GenerateSmallWorldMsg() 
	g.generateWorlds()