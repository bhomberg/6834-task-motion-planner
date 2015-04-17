#!/usr/bin/python

DIR = '../states/'

class StateGeneratorSmall:
	def __init__(self):
		self.NUMBLOCKS = 10

	# Concatenate substrings & add numbers between them, do this for the number of blocks
	# strs = substrings to concatenate (with number between each substring)
	# the final character of the return strign
	# whether or not to add a number after the last substring
	# the block # to start at
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

	def generateString(self, surface_ids, initialPoses, goals):
		s = ''
		# environment
		s += self.concatMult(["BLOCK"," - physob"],",")
		s += "LEFTARM - gripper,"
		s += self.concatMult(["GP_BLOCK"," - pose"],",")
		for s_id in surface_ids:
			s2 = "_" + s_id + " - pose"
			s += self.concatMult(["PDP_BLOCK",s2],",")
		s += "INITPOSE - pose,"

		for i in range(len(surface_ids)):
			s += surface_ids[i] + " - location"
			if i < len(surface_ids) - 1:
				s += ","
			else:
				s += "\n"
		
		# initial conditions
		s += "ROBOTAT INITPOSE,EMPTY LEFTARM,"
		s += self.concatMult(["ISGPFG GP_BLOCK", " BLOCK"],",",True)
		for s_id in surface_ids:
			middle = "_" + s_id + " BLOCK"
			end = " " + s_id
			s += self.concatMult(["ISGPFPD PDP_BLOCK",middle, end],",")
		for s_id in surface_ids:
			s2 = "ISLFPD " + s_id + " BLOCK"
			s += self.concatMult([s2],",",True)
		# initial block locations
		for block in initialPoses:
			s += "AT " + block[0] + " " + block[1] + ","
		# remove last comma from string
		s = s[:-1]
		s += "\n"

		# goal
		for block in goals:
			s += "AT " + block[0] + " " + block[1] + ","
		# remove last comma from string
		s = s[:-1]
		s += "\n"

		# initial pose
		s += "INITPOSE"
		return s

	def generateFile(self,filename,surface_ids,initialPoses,goals):
		s = self.generateString(surface_ids,initialPoses,goals)
		f = open(DIR+filename, 'w')
		f.write(s)
		f.close()

	def genStateFromWorld(self,world,filename):
		self.NUMBLOCKS = len(world.world.movable_objects)
		self.generateFile(filename)

	def generateStates(self):
		self.NUMBLOCKS = 3
		# state3Blocks_1
		filename = "state3Blocks_1"
		initialPoses = [("BLOCK0","S"),("BLOCK1","S"),("BLOCK2","I")]
		goals = [("BLOCK0","I"),("BLOCK1","I"),("BLOCK2","S")]
		self.generateFile(filename,["I","S"],initialPoses,goals)
		# state3Blocks_2
		filename = "state3Blocks_2"
		initialPoses = [("BLOCK0","I"),("BLOCK1","I"),("BLOCK2","A")]
		goals = [("BLOCK0","S"),("BLOCK2","S")]
		self.generateFile(filename,["I","A","S"],initialPoses,goals)
		# state3Blocks_3
		filename = "state3Blocks_3"
		initialPoses = [("BLOCK0","I"),("BLOCK1","I"),("BLOCK2","I")]
		goals = [("BLOCK0","S"),("BLOCK1","S"),("BLOCK2","S")]
		self.generateFile(filename,["I","A","S"],initialPoses,goals)
		# state3Blocks_4
		filename = "state3Blocks_4"
		initialPoses = [("BLOCK0","I"),("BLOCK1","I"),("BLOCK2","I")]
		goals = [("BLOCK0","S"),("BLOCK1","S"),("BLOCK2","S")]
		self.generateFile(filename,["I","A","S"],initialPoses,goals)
		
		NUMBLOCKS = 4

		# state4Blocks_1
		filename = "state4Blocks_1"
		initialPoses = [("BLOCK0","I"),("BLOCK1","I"),("BLOCK2","I"),("BLOCK3","S")]
		goals = [("BLOCK0","S"),("BLOCK1","S"),("BLOCK2","S"),("BLOCK3","I")]
		self.generateFile(filename,["I","S"],initialPoses,goals)
		# state4Blocks_2
		filename = "state4Blocks_2"
		initialPoses = [("BLOCK0","I"),("BLOCK1","I"),("BLOCK2","S"),("BLOCK3","S")]
		goals = [("BLOCK0","I"),("BLOCK1","S"),("BLOCK2","I"),("BLOCK3","S")]
		self.generateFile(filename,["I","S"],initialPoses,goals)
		# state4Blocks_3
		filename = "state4Blocks_3"
		initialPoses = [("BLOCK0","I"),("BLOCK1","I"),("BLOCK2","I"),("BLOCK3","I")]
		goals = [("BLOCK0","A"),("BLOCK1","S"),("BLOCK2","A"),("BLOCK3","S")]
		self.generateFile(filename,["I","A","S"],initialPoses,goals)
		# state4Blocks_4
		filename = "state4Blocks_4"
		initialPoses = [("BLOCK0","I"),("BLOCK1","I"),("BLOCK2","I"),("BLOCK3","A")]
		goals = [("BLOCK0","A"),("BLOCK1","S"),("BLOCK2","A"),("BLOCK3","S")]
		self.generateFile(filename,["I","A","S"],initialPoses,goals)

if __name__ == "__main__":
	# single argument: Number of blocks
	stateGen = StateGeneratorSmall()
	stateGen.generateStates()