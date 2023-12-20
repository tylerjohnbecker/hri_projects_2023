#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from week7.msg import Animate

class Follower:

	def __init__(self):

		self.speach_sub = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_cb)

		self.command_pub = rospy.Publisher('/animate', Animate, queue_size=10)

	def nod(self):
		
		nod_down = Animate()
		nod_down.pose = 3
		nod_down.duration = .7

		nod_up = Animate()
		nod_up.pose = 4
		nod_up.duration = .7

		for i in range(3):
			self.command_pub.publish(nod_down)

			self.command_pub.publish(nod_up)


	def wave(self):

		wave_left = Animate()
		wave_left.pose = 6
		wave_left.duration = .7

		wave_right = Animate()
		wave_right.pose = 5
		wave_right.duration = .7

		for i in range(3):
			self.command_pub.publish(wave_left)
			
			self.command_pub.publish(wave_right)

	def speech_cb(self, data):

		if 'wave' in data.data:
			self.wave()
		elif 'nod' in data.data:
			self.nod()

if __name__ == '__main__':

	rospy.init_node('follow_commands', anonymous=False)

	fol = Follower()

	rospy.spin()