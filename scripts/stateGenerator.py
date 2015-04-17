#!/usr/bin/python

import sys

# The number of blocks (including block 0)
# set by command line args


# Concatenate substrings & add numbers between them, do this for the number of blocks
# strs = substrings to concatenate (with number between each substring)
# the final character of the return strign
# whether or not to add a number after the last substring
# the block # to start at
class StateGenerator:
	def __init__(self):
		self.NUMBLOCKS = 10

	def concatMult(self,strs,last,end=False,start=0):
		s = ''
		r = len(strs) if end else len(strs)-1 
		for i in range(start, self.NUMBLOCKS):
			for x in range(r):
				s += strs[x]+str(i)
			if r < len(strs):
				s += strs[-1]
			if not i == self.NUMBLOCKS-1:
				s += ","
		s += last
		return s

	def generateString(self):
		s = ''
		# environment
		s += self.concatMult(["BLOCK"," - physob"],",")
		s += "LEFTARM - gripper,"
		s += self.concatMult(["GP_BLOCK"," - pose"],",")
		s += self.concatMult(["PDP_BLOCK","_S - pose"],",")
		s += self.concatMult(["PDP_BLOCK","_I - pose"],",")
		s += "INITPOSE - pose,"
		s += "S - location,"
		s += "I - location\n"
		# initial conditions
		s += "ROBOTAT INITPOSE,EMPTY LEFTARM,"
		s += self.concatMult(["ISGPFG GP_BLOCK", " BLOCK"],",",True)
		s += self.concatMult(["ISGPFPD PDP_BLOCK","_S BLOCK", " S"],",")
		s += self.concatMult(["ISGPFPD PDP_BLOCK","_I BLOCK", " I"],",")
		s += self.concatMult(["ISLFPD S BLOCK"],",",True)
		s += self.concatMult(["ISLFPD I BLOCK"],",",True)
		s += self.concatMult(["AT BLOCK", " I"],"\n")
		# goal
		s += "AT BLOCK0 S\n"
		# initial pose
		s += "INITPOSE"

		return s

	def generateFile(self,filename):
		s = self.generateString()
		f = open(filename, 'w')
		f.write(s)
		f.close()

	def genStateFromWorld(self,world,filename):
		self.NUMBLOCKS = len(world.world.movable_objects)
		self.generateFile(filename)

if __name__ == "__main__":
	# single argument: Number of blocks
	# 2 arguments: filename, number of blocks
	if len(sys.argv) == 1:
		filename = 'tmp'
		NUMBLOCKS = 10
	elif len(sys.argv) == 2:
		filename = 'tmp'
		NUMBLOCKS = int(sys.argv[1])
	else:
		filename = sys.argv[1]
		NUMBLOCKS = int(sys.argv[2])

	stateGen = StateGenerator()
	stateGen.NUMBLOCKS = 17
	stateGen.generateFile(filename)