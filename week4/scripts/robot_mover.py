#!/usr/bin/python3

import math
import rospy
import copy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from week4.msg import Group
from geometry_msgs.msg import PoseStamped

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

class Object:

	def __init__(self, midpoint, gains, q_star):
		self.midpoint 		= copy.deepcopy(midpoint)
		self.gains 			= copy.deepcopy(gains)
		self.q_star 		= copy.deepcopy(q_star)

class ObjectIdentifier:

	def __init__ (self, robot_prefix, odom):
		self.objects 		= []
		self.gains 			= np.array([.10, .10, .10])
		self.q_star			= 3.0
		self.odometry		= odom

		self.laser_sub		= rospy.Subscriber(robot_prefix + '/base_scan', LaserScan, self.laserCallback)

	def laserCallback(self, data):
		
		n_objects = []

		true_angle_min = self.odometry.orientation[2] + data.angle_min

		for index, dist in enumerate(data.ranges):

			if dist <= self.q_star:

				dist_x = dist * math.cos( true_angle_min + data.angle_increment * index )
				dist_y = dist * math.sin( true_angle_min + data.angle_increment * index )

				midpoint = np.array([self.odometry.position[0] + dist_x, self.odometry.position[1] + dist_y, self.odometry.position[2]])

				n_objects.append( Object(midpoint, self.gains, self.q_star) )

		self.objects = n_objects

class GroupIdentifier:

	def __init__(self, robot_prefix):
		self.objects 		= []
		self.gains			= np.array([1, 1, 1])
		self.buffer 		= 2.0 

		self.group_sub		= rospy.Subscriber(robot_prefix + '/detected_groups', Group, self.groupCallback)

	def groupCallback(self, data):
		
		self.objects = []

		midpoint = np.array([data.midpoint.position.x, data.midpoint.position.y, data.midpoint.position.z])

		n_obj = Object(midpoint, self.gains, data.size + self.buffer)

		self.objects.append(n_obj)			

class OdomListener:

	def __init__(self, robot_prefix):
		self.position 		= np.array([0, 0, 0])
		self.orientation	= np.array([0, 0, 0])
		self.lin_velocity	= np.array([0 ,0, 0])
		self.ang_velocity	= np.array([0 ,0, 0])

		self.odom_sub 		= rospy.Subscriber(robot_prefix + '/base_pose_ground_truth', Odometry, self.odomCallback)

	def odomCallback(self, data):
		self.position 		= np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

		( euler_x, euler_y, euler_z ) = euler_from_quaternion([ data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ])

		self.orientation 	= np.array([euler_x, euler_y, euler_z])

		self.lin_velocity	= np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
		self.ang_velocity	= np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

class RobotMover:

	def __init__(self, robot_prefix, odom):
		self.twist 			= Twist()
		self.odometry 		= odom

		self.twist_pub 		= rospy.Publisher(robot_prefix + '/cmd_vel', Twist, queue_size=1)

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

	def __init__ (self, robot_prefix, tol):
		self.groups 		= GroupIdentifier(robot_prefix)
		self.odom			= OdomListener(robot_prefix)
		self.objects 		= ObjectIdentifier(robot_prefix, self.odom)
		self.mover 			= RobotMover(robot_prefix, self.odom)

		self.goal 			= np.array([0, 0, 0])
		self.obstacles  	= []
		self.tolerance 		= tol

		self.nav_sub		= rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalCallback)

	def goalCallback (self, data):
		self.navigate(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))

	def updateObstacles(self):
		self.obstacles = []

		objects = copy.deepcopy(self.objects.objects)
		groups = copy.deepcopy(self.groups.objects)

		for i in objects:
			self.obstacles.append(i)

		for i in groups:
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

		self.mover.setTwistFromVelocity(np.array([0, 0, 0]), 1)

		self.mover.publishTwist()

def main():

	pfc = PotentialFieldController(robot_prefix='/robot_0', tol=.5)

	while not rospy.is_shutdown():
		pass

if __name__ == '__main__':

	rospy.init_node('intelligent_mover', anonymous=True)

	main()