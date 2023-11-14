#!/usr/bin/python3

import math
import rospy
import copy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Object:

	def __init__(self, midpoint, gains, q_star):
		self.midpoint 		= copy.deepcopy(midpoint)
		self.gains 			= copy.deepcopy(gains)
		self.q_star 		= copy.deepcopy(q_star)

class ObjectIdentifier:

	def __init__ (self, odom):
		self.objects 		= []
		self.gains 			= np.array([.10, .10, .10])
		self.q_star			= 3.0
		self.odometry		= odom

		self.laser_sub		= rospy.Subscriber('/robot_0/base_scan', LaserScan, self.laserCallback)

	def laserCallback(self, data):
		
		n_objects = []

		# print('hello')

		true_angle_min = self.odometry.orientation[2] + data.angle_min

		for index, dist in enumerate(data.ranges):

			if dist <= self.q_star:

				dist_x = dist * math.cos( true_angle_min + data.angle_increment * index )
				dist_y = dist * math.sin( true_angle_min + data.angle_increment * index )

				midpoint = np.array([self.odometry.position[0] + dist_x, self.odometry.position[1] + dist_y, self.odometry.position[2]])

				# print(self.odometry.position, dist, (true_angle_min + data.angle_increment * index), dist_x, dist_y, midpoint)

				n_objects.append( Object(midpoint, self.gains, self.q_star) )

		self.objects = n_objects


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

		true_angle = math.atan2(vel[1], vel[0])

		dist, direction = get_smallest_dist_and_direction( self.odometry.orientation[2] , true_angle )

		self.twist.linear.x = max(np.linalg.norm(vel) * math.cos(dist), 0)

		if direction == 'right':
			self.twist.angular.z = -1 * dist / delta_t
		else:
			self.twist.angular.z = dist / delta_t

class PotentialFieldController:

	tolerance = .5

	def __init__ (self, tol):
		self.groups 		= GroupIdentifier()
		self.odom			= OdomListener()
		self.objects 		= ObjectIdentifier(self.odom)
		self.mover 			= RobotMover(self.odom)

		self.goal 			= np.array([0, 0, 0])
		self.obstacles  	= []
		self.tolerance 		= tol

	def updateObstacles(self):
		self.obstacles = []

		objects = copy.deepcopy(self.objects.objects)

		for i in objects:
			self.obstacles.append(i)

	def calculateForce(self):
		
		tot_force	= np.array([0., 0., 0.])
		rep_force 	= np.array([0., 0., 0.])
		attr_force 	= np.array([0., 0., 0.])

		attr_gains 	= np.array([1.0, 1.0, 1.0])

		# equations for attractive and repulsive force are taken from here: https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
		attr_force 	+= attr_gains * (self.goal - self.odom.position)

		# max range for repulsive force of the objects is 1 meter
		q_star = 1.0

		for i in self.obstacles:
			dist = np.linalg.norm(self.odom.position - i.midpoint)

			rep_force += i.gains * ( (1 / i.q_star) - (1 / dist) ) * (1 / math.pow(dist, 2) ) * ( (self.odom.position - i.midpoint) / dist )

		tot_force = attr_force - rep_force

		return tot_force

	def navigate(self, target):

		self.goal = copy.deepcopy(target)

		delta_t = .2

		while np.linalg.norm(self.goal - self.odom.position) >= self.tolerance:

			self.updateObstacles()

			next_delta_v = self.calculateForce()

			self.mover.setTwistFromVelocity(next_delta_v, delta_t)

			self.mover.publishTwist()

def main():
	pass

if __name__ == '__main__':

	rospy.init_node('intelligent_mover', anonymous=True)

	pfc = PotentialFieldController(tol=.5)

	pfc.navigate(np.array([-6, 6, 0]))