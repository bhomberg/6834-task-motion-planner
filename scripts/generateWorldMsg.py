#!/usr/bin/python

import roslib
from task_motion_planner.msg import *

MIN_BLOCK_GRID_SIZE = 3
MAX_BLOCK_GRID_SIZE = 17

class generateWorldMsg:
	def __init__(self):
		self.WORLD_GRID_SIZE = 17
		self.CENTER = self.WORLD_GRID_SIZE/2 + 1

	def generateWorld(self,block_shape,shape_size):
		my_world = world()
		surfaceS = obj()
		surfaceS.id = 'S'
		surfaceI = obj()
		surfaceI.id = 'I'
		my_world.surfaces = [surfaceS,surfaceI]
		if block_shape=='SQUARE':
			self.generateBlockGrid(my_world,shape_size)
		elif block_shape=='CROSS':
			self.generateBlockCross(my_world,shape_size)		
		elif block_shape=='HLINE':
			self.generateBlockHLine(my_world,shape_size)		
		elif block_shape=='VLINE':
			self.generateBlockVLine(my_world,shape_size)
		print my_world.moveable_objects

	def generateBlockGrid(self,my_world,block_grid_size):
		idx = 1
		bound = block_grid_size/2
		for j in xrange(-bound,bound+1):
			for i in xrange(-bound,bound+1):
				idx = self._add_block(my_world,i,j,idx)

	def generateBlockCross(self,my_world,block_grid_size):
		idx = 1
		bound = block_grid_size/2
		for j in xrange(-bound,bound+1):
			for i in xrange(-bound,bound+1):
				if i==0 or j==0:
					idx = self._add_block(my_world,i,j,idx)	

	def generateBlockHLine(self,my_world,length):
		idx = 1
		bound = length/2
		for i in xrange(-bound,bound+1):
			idx = self._add_block(my_world,i,0,idx)	

	def generateBlockVLine(self,my_world,length):
		idx = 1
		bound = length/2
		for i in xrange(-bound,bound+1):
			idx = self._add_block(my_world,0,i,idx)

	def _add_block(self,my_world,i,j,idx):
		o = obj()
		o.id = 'BLOCK' + str(idx)
		# green block
		if i==0 and j==0:
			o.id = 'BLOCK' + str(0)
		else:
			idx += 1
		o.loc.surface_id = 'I'
		o.loc.x = self.CENTER + i
		o.loc.y = self.CENTER + j
		my_world.moveable_objects.append(o)
		return idx
if __name__ == "__main__":
	g = generateWorldMsg()
	for i in xrange(MIN_BLOCK_GRID_SIZE,MAX_BLOCK_GRID_SIZE+1,2):
		g.generateWorld("SQUARE",i)
	g.generateWorld("CROSS",17)
	g.generateWorld("HLINE",16)
	g.generateWorld("VLINE",16)

