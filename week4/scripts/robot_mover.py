#!/usr/bin/python3

import numpy as np

class Object:

	def __init__(self):
		self.midpoint 		= np.array([0, 0, 0])
		self.gains 			= np.array([0, 0, 0])

class ObjectIdentifier:

	def __init__ (self):
		self.objects 		= []

	def laserCallback(self, data):
		pass

class GroupIdentifier:

	def __init__(self):
		self.objects 		= []

	def groupCallback(self, data):
		pass

class OdomListener:

	def __init__(self):
		self.position 		= np.array([0, 0, 0])
		self.lin_velocity	= np.array([0 ,0, 0])
		self.ang_velocity	= np.array([0 ,0, 0])

	def odomCallback(self, data):
		pass

class RobotMover:

	def __init__(self):
		pass

	def publishTwist(self):
		pass

	def setTwistFromForceVector(self, force):
		pass

class PotentialFieldController:

	tolerance = .5

	def __init__ (self):
		self.groups 		= GroupIdentifier()
		self.objects 		= ObjectIdentifier()
		self.odom			= OdomListener()
		self.mover 			= RobotMover()

		self.obstacles  	= np.array([0, 0, 0])
		self.goal 			= np.array([0, 0, 0])

	def calculateForce(self):
		
		tot_force	= np.array([0, 0, 0])
		rep_force 	= np.array([0, 0, 0])
		attr_force 	= np.array([0, 0, 0])

		attr_gains 	= np.array([1.0, 1.0, 1.0])

		# equations for attractive and repulsive force are taken from here: https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
		attr_force 	+= attr_gains * (self.goal - self.odom.position)

		# max range for repulsive force of the objects is 1 meter
		q_star = 1.0

		for i in self.obstacles:
			dist = np.linalg.norm(self.odom.position - i.midpoint)

			rep_force += i.gains * ( (1 / q_star) - (1 / dist) ) * (1 / math.pow(dist, 2)) * self.odom.lin_velocity

		tot_force = rep_force + attr_force

		return -1 * tot_force

	def navigate(self, target):

		self.goal = copy.deepcopy(target)

		while np.linalg.norm(self.goal - self.odom.position) <= tolerance:

			next_force = self.calculateForce()

			self.mover.setTwistFromForceVector(next_force)



def main():
	pass

if __name__ == '__main__':
	print('hello world!')