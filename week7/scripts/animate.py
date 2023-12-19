#!/usr/bin/python3

import rospy
import copy

from week7.msg import Animate, AnimatePose
from sensor_msgs.msg import JointState

def lerp (start, finish, max_steps, step_num):
	return start + (finish - start) * (step_num / max_steps) 

class Animator:

	poses = [ [-0.4359050299999998, 0.1950229320000001, -0.6781258183999999, -0.03759168099999999, -1.005472138, 0.22341061528, -0.29035266030000006, 0.4890879553, -0.6781258183999999, 0.019458308500000077, \
			-0.3924415929999998, 0.7792603338699999, 0.4560226418000002, 0.3082710449, 1.7920076639999998, 1.0956163137, -1.6314110739999999, -1.4406007467400002, 1.3416387720000003, 0.6387, -0.7216418199999999, \
			-1.2749348774, 1.0891368739999998, 0.63894293134, -1.0217319740000002, 0.8014, 0.8013190586, 0.8013190586, 0.8013190586, 0.6386354913000001, 0.6386354913000001, 0.6386354913000001, 0.8013190586, \
			0.6386354913000001, 0.8013190586, 0.6386354913000001, 0.8013190586, 0.6386354913000001, 0.6386354913000001, 0.8013190586, 0.8013190586, 0.6386354913000001], \
			[0.8709757919999999, 0.21769127040000014, -1.0362787376, 0.5113230530000001, -0.06946437999999988, 1.1809890872500002, -0.9406439262, -0.23569900829999998, -1.0362787376, -0.6609526235000001,\
			 -0.8097467349999999, 1.6162319847100002, -0.6914637183999999, -0.19028251200000001, 1.3556854999999999, 0.7118778685999998, -1.567589572, -1.44920611312, -1.494114304, 0.1578, 1.60179456, -0.0794230991,\
			  1.40782725, 1.22561755858, 1.4138640239999998, 0.4815, 0.4814513685, 0.4814513685, 0.4814513685, 0.1577840622, 0.1577840622, 0.1577840622, 0.4814513685, 0.1577840622, 0.4814513685, 0.1577840622, \
			  0.4814513685, 0.1577840622, 0.1577840622, 0.4814513685, 0.4814513685, 0.1577840622], \
			[0.15809378600000024, -0.5371287936, 0.5102478224, 0.7233080270000001, -1.44297598, 0.17291891136999998, -0.3997553481, -0.24118274739999998, 0.5102478224, 0.1810208080000001, -0.7018856769999999, \
			1.53729735589, -0.7683582261999999, -0.048405347200000004, -2.045625136, -0.06691922370000003, 1.1679751999999999, -0.53733921952, -1.033769516, 0.1221, 0.31535330399999983, -0.8068691296999999, \
			-0.977762096, 1.0876297538200002, -0.17399719800000013, 0.1684, 0.1683829916, 0.1683829916, 0.1683829916, 0.1220876679, 0.1220876679, 0.1220876679, 0.1683829916, 0.1220876679, 0.1683829916, 0.1220876679, \
			0.1683829916, 0.1220876679, 0.1220876679, 0.1683829916, 0.1683829916, 0.1220876679] ]

	def __init__(self, max_frames):

		self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

		self.animate_sub = rospy.Subscriber('/animate', Animate, self.animateCb)

		self.animate_pose_sub = rospy.Subscriber('/animatePose', AnimatePose, self.animatePoseCb)

		self.joints = []
		self.current_pose = []

		self.frames = max_frames

		joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.jointCb)

		while self.current_pose == []:
			pass

	def jointCb(self, data):
		self.joints = copy.deepcopy(data.name)
		self.current_pose = copy.deepcopy(data.position)

	def poseToJointStateMsg(self, pose):

		to_ret = JointState()

		to_ret.position = pose
		to_ret.name = self.joints

		to_ret.header.stamp = rospy.Time.now()
		to_ret.header.frame_id = 'base_link'

		return to_ret

	def animatePoseCb(self, data):

		rospy.loginfo('Animate Pose callback called!')

		self.current_pose = []

		joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.jointCb)

		while self.current_pose == []:
			pass

		start_pose = copy.deepcopy(self.current_pose)
		goal_pose = data.goal_pose.position

		time_inc = data.duration / self.frames

		for i in range(self.frames):

			next_pose = [ lerp(start_pose[j], goal_pose[j], self.frames, i) for j in range(len(start_pose)) ]

			self.joint_state_pub.publish(self.poseToJointStateMsg(next_pose))

			rospy.Time.sleep(time_inc)

	def animateCb(self, data):

		rospy.loginfo('Animate callback called!')

		self.current_pose = []

		joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.jointCb)

		while self.current_pose == []:
			pass

		start_pose = copy.deepcopy(self.current_pose)
		goal_pose = Animator.poses[data.pose]

		# estimate of how long it take to get and publish the next frame
		epsilon = .001

		time_inc = data.duration / self.frames - epsilon

		for i in range(self.frames):

			next_pose = [ lerp(start_pose[j], goal_pose[j], self.frames, i) for j in range(len(start_pose)) ]

			self.joint_state_pub.publish(self.poseToJointStateMsg(next_pose))

			rospy.sleep(time_inc)


if __name__ == '__main__':

	rospy.init_node('animator', anonymous=True)

	anim = Animator(100)

	while not rospy.is_shutdown():
		pass