#!/usr/bin/python3

import rospy
from std_msgs.msg import String

saved_dialogue = ''

class Dialogue:

	def __init__(self):

		self.saved_dialogue = ''
		self.chatter_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
		self.dialogue_sub = rospy.Subscriber('/speech_recognition/final_result', String, self.dialogue_cb)
		self.start_sub = rospy.Subscriber('/go', String, self.run)

	def dialogue_cb (self,data):
		self.saved_dialogue = data.data

	def run(self, data):
		first_question = String()
		first_question.data = 'Do you like to play with dogs?'

		second_question = String()
		second_question.data = 'Do you like robots?'

		third_question = String()
		third_question.data = 'Would you like some candy that I found?'

		num_negative = 0

		self.chatter_pub.publish(first_question)

		self.saved_dialogue = ''

		while self.saved_dialogue == '':
			pass

		if 'yes' in self.saved_dialogue:
			first_response_positive = String()
			first_response_positive.data = 'Hey that\'s awesome I like dogs to there was this one time when I saw a dog and i though wow that\'s a lot of legs'

			self.chatter_pub.publish(first_response_positive)
		else:
			first_response_negative = String()
			first_response_negative.data = 'I think you are missing out because dogs are awesome'

			self.chatter_pub.publish(first_response_negative)


		rospy.sleep(3)

		self.chatter_pub.publish(second_question)

		self.saved_dialogue = ''

		while self.saved_dialogue == '':
			pass

		if 'yes' in self.saved_dialogue:
			second_response_positive = String()
			second_response_positive.data = 'Robots are really cool like how I am also cool like other robots are cool like how I am cool'

			self.chatter_pub.publish(second_response_positive)
		else:
			second_response_negative = String()
			second_response_negative.data = 'Robots are really cool like how I am also cool like other robots are cool like how I am cool'

			self.chatter_pub.publish(second_response_negative)

		rospy.sleep(3)

		self.chatter_pub.publish(third_question)

		self.saved_dialogue = ''

		while self.saved_dialogue == '':
			pass

		if 'yes' in self.saved_dialogue:
			second_response_positive = String()
			second_response_positive.data = 'oh I actually don\'t have any candy I just thought you might like it if I did and gave it to you'

			self.chatter_pub.publish(second_response_positive)
		else:
			second_response_negative = String()
			second_response_negative.data = 'No worries dude cavities and dentists am I right, all right'

			self.chatter_pub.publish(second_response_negative)

		rospy.loginfo("hello world!")



if __name__ == '__main__':
	rospy.init_node('dialogue', anonymous=False)

	d = Dialogue()

	rospy.spin()