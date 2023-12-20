#!/usr/bin/python3

import rospy

from std_msgs.msg import String

class Repeater:

	def __init__ (self):

		self.chatter_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)

		self.speach_sub = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_cb)

	def speech_cb(self, data):

		msg = String()

		msg.data = data.data

		self.chatter_pub.publish(msg)


if __name__ == '__main__':

	rospy.init_node("ChatterRepeater", anonymous=False)

	rp = Repeater()

	rospy.spin()