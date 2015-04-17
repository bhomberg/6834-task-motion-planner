#!/usr/bin/python

import sys

# The number of blocks (including block 0)
# set by command line args
NUMBLOCKS = 10

# Concatenate substrings & add numbers between them, do this for the number of blocks
# strs = substrings to concatenate (with number between each substring)
# the final character of the return strign
# whether or not to add a number after the last substring
# the block # to start at
def concatMult(strs,last,end=False,start=0):
	s = ''
	r = len(strs) if end else len(strs)-1 
	for i in range(start, NUMBLOCKS):
		for x in range(r):
			s += strs[x]+str(i)
		if r < len(strs):
			s += strs[-1]
		if not i==NUMBLOCKS-1:
			s += ","
	s += last
	return s
def generateString():
	s = ''
	# environment
	s += concatMult(["BLOCK"," - physob"],",")
	s += "LEFTARM - gripper,"
	s += concatMult(["GP_BLOCK"," - pose"],",")
	s += concatMult(["PDP_BLOCK","_S - pose"],",")
	s += concatMult(["PDP_BLOCK","_I - pose"],",")
	s += "INITPOSE - pose,"
	s += "S - location,"
	s += "I - location\n"
	# initial conditions
	s += "ROBOTAT INITPOSE,EMPTY LEFTARM,"
	s += concatMult(["ISGPFG GP_BLOCK", " BLOCK"],",",True)
	s += concatMult(["ISGPFPD PDP_BLOCK","_S BLOCK", " S"],",")
	s += concatMult(["ISGPFPD PDP_BLOCK","_I BLOCK", " I"],",")
	s += concatMult(["ISLFPD S BLOCK"],",",True)
	s += concatMult(["ISLFPD I BLOCK"],"\n",True)
	# goal
	s += "AT BLOCK0 S\n"
	# initial pose
	s += "INITPOSE"

	return s

def generateFile(filename):
	s = generateString()
	f = open(filename, 'w')
	f.write(s)
	f.close()

if __name__ == "__main__":
	# single argument: Number of blocks
	# 2 arguments: filename, number of blocks
	if len(sys.argv) == 1:
		filename = 'state'
		NUMBLOCKS = 10
	if len(sys.argv) == 2:
		filename = 'state'
		NUMBLOCKS = int(sys.argv[1])
	else:
		filename = sys.argv[1]
		NUMBLOCKS = int(sys.argv[2])

	generateFile(filename)