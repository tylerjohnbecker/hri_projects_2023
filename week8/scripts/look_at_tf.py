#!/usr/bin/python3

import rospy
import tf2_ros
import math
import copy

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from week7.msg import AnimatePose

class Looker:

	def __init__(self):
		self.animation_pub = rospy.Publisher('/animatePose', AnimatePose, queue_size=10)

		self.req_sub = rospy.Subscriber('/look_at', String, self.lookCb)
		
		self.name = []
		self.position = []
		self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.jointCb)

		self.tf_buf = tf2_ros.Buffer()
		self.list = tf2_ros.TransformListener(self.tf_buf )

		while self.name == [] and self.position == []:
			pass

		self.ind_pitch = -1
		self.ind_yaw = -1

		for index, val in enumerate(self.name):
			if val == 'HeadPitch':
				self.ind_pitch = index
			elif val == 'HeadYaw':
				self.ind_yaw = index

	def jointCb(self, data):
		self.position = list(data.position)
		self.name = list(data.name)

	def lookCb(self, data):

		# lookup tf from tf2
		transform = None

		try:
			transform = self.tf_buf.lookup_transform('Neck', data.data, rospy.Time.now(), rospy.Duration(1.0))
		# taken from http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rospy.logfatal(f"Error when looking up transform {data.data}")
			return

		# calculate joint angles
		pitch = -1 * math.atan2(transform.transform.translation.z, transform.transform.translation.x)
		yaw = math.atan2(transform.transform.translation.y, transform.transform.translation.x)

		# update joint state
		pos = copy.deepcopy(self.position)

		pos[self.ind_pitch] = pitch
		pos[self.ind_yaw] += yaw

		# send to animation server
		ap = AnimatePose()

		ap.goal_pose.position = pos
		ap.duration = 1.0

		self.animation_pub.publish(ap)

if __name__ == '__main__':

	rospy.init_node('lookTf', anonymous=False)

	looky = Looker()

	rospy.loginfo("Server up and running!")

	rospy.spin()