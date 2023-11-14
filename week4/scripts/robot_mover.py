#!/usr/bin/python3

import math
import rospy
import copy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

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
		self.orientation	= np.array([0, 0, 0])
		self.lin_velocity	= np.array([0 ,0, 0])
		self.ang_velocity	= np.array([0 ,0, 0])

		self.odom_sub 		= rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, self.odomCallback)

	def odomCallback(self, data):
		self.position 		= np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

		( euler_x, euler_y, euler_z ) = euler_from_quaternion([ data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ])

		self.orientation 	= np.array([euler_x, euler_y, euler_z])

		self.lin_velocity	= np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
		self.ang_velocity	= np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

def get_smallest_dist_and_direction(ang1, ang2):

    temp1 = ang1
    temp2 = ang2

    if ang1 <= 0:
        temp1 = math.pi * 2 + ang1

    if ang2 <= 0:
        temp2 = math.pi * 2 + ang2

    distance1 = temp1 - temp2
    distance2 = temp2 - temp1

    if distance1 < 0:
        distance1 += math.pi * 2

    if distance2 < 0:
        distance2 += math.pi * 2

    if distance1 >= distance2:
        return distance2, 'left'
    else:
        return distance1, 'right'

class RobotMover:

	def __init__(self, odom):
		self.twist 			= Twist()
		self.odometry 		= odom

		self.twist_pub 		= rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)

	def publishTwist(self):
		try:
			# print(self.twist)
			self.twist_pub.publish(self.twist)

		except rospy.ROSInterruptException:
			rospy.logfatal('Unable to publish cmd_vel message...')

	def setTwistFromVelocity(self, vel, delta_t):
		self.twist.linear.x = np.linalg.norm(vel)

		true_angle = math.atan2(vel[1], vel[0])

		print(self.odometry.orientation[2])

		dist, direction = get_smallest_dist_and_direction( self.odometry.orientation[2] , true_angle )

		print(dist)

		if direction == 'right':
			self.twist.angular.z = -1 * dist / delta_t
		else:
			self.twist.angular.z = dist / delta_t

class PotentialFieldController:

	tolerance = .5

	def __init__ (self):
		self.groups 		= GroupIdentifier()
		self.objects 		= ObjectIdentifier()
		self.odom			= OdomListener()
		self.mover 			= RobotMover(self.odom)

		self.goal 			= np.array([0, 0, 0])
		self.obstacles  	= []

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

	def calculateVelocity(self, delta_t):
		# assuming that mass is 1 for simplicity delta_v = delta_t * force / mass
		return delta_t * self.calculateForce()

	def navigate(self, target):

		self.goal = copy.deepcopy(target)

		delta_t = .2

		for i in range(10000):
		# while np.linalg.norm(self.goal - self.odom.position) <= tolerance:

			# next_delta_v = self.calculateVelocity(delta_t)

			self.mover.setTwistFromVelocity(np.array([0, 1, 0]), delta_t)

			self.mover.publishTwist()

def main():
	pass

if __name__ == '__main__':

	rospy.init_node('intelligent_mover', anonymous=True)

	pfc = PotentialFieldController()

	pfc.navigate([0, 0, 0])