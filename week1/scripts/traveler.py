#!/usr/bin/env python3

import rospy
import copy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import threading

my_pos = None

def spin_thread():
	rospy.spin()

	# print('hello world!')

def set_pos_cb(data):
	global my_pos 

	my_pos = data.pose.pose

	# print("callback_called")

def go_forward(pub, dist):

	twistmsg = Twist()

	print("going forward!")

	try:
		twistmsg.linear.x = 1.0

		initial_pos = copy.deepcopy(my_pos.position)

		initial_pos = [ initial_pos.x, initial_pos.y, initial_pos.z ]

		# until we've moved the specified square side length
		while math.dist(initial_pos, [ my_pos.position.x, my_pos.position.y, my_pos.position.z ]) <= dist:

			pub.publish(twistmsg)

	
	except rospy.ROSInterruptException:
		rospy.loginfo("please stop")

def is_same_angle(ang1, ang2, eps):
	temp1 = ang1
	temp2 = ang2

	if ang1 <= 0:
		temp1 = math.pi * 2 + ang1

	if ang2 <= 0:
		temp2 = math.pi * 2 + ang2

	distance1 = temp1 - temp2
	distance2 = temp2 - temp1

	if distance1 <= 0:
		distance1 += math.pi * 2

	if distance2 <= 0:
		distance2 += math.pi * 2

	if distance1 >= distance2:
		return distance2 <= eps
	else:
		return distance1 <= eps


def turn_right(pub, rad, forward_speed):
	twistmsg = Twist()

	print("turning right!")

	try:
		(init_r, init_p, init_y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ])

		end_y = init_y - rad

		cur_y = init_y

		if end_y >= math.pi:
			end_y += math.pi * 2

		print(f'{end_y}!!!!')

		twistmsg.angular.z = -1 * math.pi / 8
		twistmsg.linear.x = forward_speed


		# was lazy so found smallest angle approach
		while not is_same_angle(cur_y, end_y, .08):

			(r, p, y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ] )

			cur_y = y

			# print(f'cur_y{cur_y} - end_y{end_y} == {end_y - cur_y}')

			pub.publish(twistmsg)


	except rospy.ROSInterruptException:
		rospy.loginfo("Oh god!")


def turn_left(pub, rad, forward_speed):
	twistmsg = Twist()

	print("turning left!")

	try:
		(init_r, init_p, init_y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ])

		end_y = init_y + rad

		cur_y = init_y

		if end_y >= math.pi:
			end_y -= math.pi * 2
		
		twistmsg.angular.z = math.pi / 8
		twistmsg.linear.x = forward_speed

		# was lazy so found smallest angle approach
		while not is_same_angle(cur_y, end_y, .08):

			(r, p, y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ] )

			cur_y = y

			pub.publish(twistmsg)


	except rospy.ROSInterruptException:
		rospy.loginfo("Oh god!")

def square_motion (pub, side_len):
	global my_pos

	while my_pos == None:
		pass

	print("Moving in a square!")

	go_forward(pub, side_len)

	turn_right(pub,math.pi / 2, 0)

	go_forward(pub, side_len)

	turn_right(pub,math.pi / 2, 0)
	
	go_forward(pub, side_len)

	turn_right(pub,math.pi / 2, 0)
	
	go_forward(pub, side_len)

	turn_right(pub,math.pi / 2, 0)

	print("ALL DONE!")


def figure_eight_motion (pub):

	global my_pos

	while my_pos == None:
		pass

	print("Moving in a figure eight!")

	turn_right(pub, math.pi / 2, 0)

	turn_left(pub, math.pi, .2)

	turn_right(pub, math.pi, .2)

	turn_right(pub, math.pi, .2)

	turn_left(pub, math.pi, .2)

	turn_left(pub, math.pi / 2, 0)


def triangle_motion (pub, side_len):

	global my_pos

	while my_pos == None:
		pass

	print("Moving in a triangle!")

	go_forward(pub, side_len)

	turn_right(pub, math.pi * 2 / 3, 0)

	go_forward(pub, side_len)

	turn_right(pub, math.pi * 2 / 3, 0)

	go_forward(pub, side_len)

	turn_right(pub, math.pi * 2 / 3, 0)


if __name__ == '__main__':


	rospy.init_node('traveler', anonymous=True)

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	
	odom_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, set_pos_cb)

	spinner = threading.Thread(target=spin_thread)

	# spinner.start()

	# square_motion(pub, 1.0)

	# triangle_motion(pub, 1.0)

	figure_eight_motion(pub)