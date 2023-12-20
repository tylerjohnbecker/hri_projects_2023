#!/usr/bin/python3

import rospy
import tf2_ros

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

def main ():
	# send look at message to the transform
	pub = rospy.Publisher('/look_at', String, queue_size=10)

	msg = String()

	msg.data = 'r_hand_point'

	for i in range(10):
		pub.publish(msg)

if __name__ == '__main__':

	rospy.init_node('lookyLoo', anonymous=False)

	main()