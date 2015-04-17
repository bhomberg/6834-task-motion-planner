#!/usr/bin/python

import roslib
from task_motion_planner.msg import *

class GenerateSmallWorldMsg:
	def __init__(self):
		self.WORLD_GRID_SIZE = 17
		self.CENTER = self.WORLD_GRID_SIZE/2
		self.block_idx = 0
		self.wall_idx = 0

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
			o.id = "WALL" + str(self.wall_idx)
			self.wall_idx += 1
			o.loc.surface_id = surface_id
			o.loc.x = self.CENTER - 1
			o.loc.y = self.CENTER + i
			my_world.walls.append(o)
			# right wall
			o = obj()
			o.id = "WALL" + str(self.wall_idx)
			self.wall_idx += 1
			o.loc.surface_id = surface_id
			o.loc.x = self.CENTER + 1
			o.loc.y = self.CENTER + i
			my_world.walls.append(o)
		# top wall
		o = obj()
		o.id = "WALL" + str(self.wall_idx)
		o.loc.surface_id = surface_id
		o.loc.x = self.CENTER
		o.loc.y = self.CENTER + ub
		my_world.walls.append(o)



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
	worlds['state3Blocks_1'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 2
	surfaces['A'] = 1
	surfaces['S'] = 0
	worlds['state3Blocks_2'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 3
	surfaces['A'] = 0
	surfaces['S'] = 0
	worlds['state3Blocks_3'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 3
	surfaces['A'] = 0
	surfaces['S'] = 0
	worlds['state3Blocks_4'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 3
	surfaces['S'] = 1
	worlds['state4Blocks_1'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 2
	surfaces['S'] = 2
	worlds['state4Blocks_2'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 4
	surfaces['A'] = 0
	surfaces['S'] = 0
	worlds['state4Blocks_3'] = g.generateWorld(surfaces)
	surfaces = dict()
	surfaces['I'] = 3
	surfaces['A'] = 1
	surfaces['S'] = 0
	worlds['state4Blocks_4'] = g.generateWorld(surfaces)
	return worlds
if __name__ == "__main__":
	g = GenerateSmallWorldMsg() 
	g.generateWorlds()