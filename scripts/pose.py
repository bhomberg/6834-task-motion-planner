class Pose:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0
		self.gripperOpen = True
	def clone(self):
		retval = Pose()
		retval.x = self.x
		retval.y = self.y
		retval.z = self.z
		retval.gripperOpen = self.gripperOpen
		return retval