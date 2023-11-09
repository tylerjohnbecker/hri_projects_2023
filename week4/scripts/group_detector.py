#!/usr/bin/python3

import rospy
import copy
import math
from tf.transformations import quaternion_from_euler
from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import Person
from geometry_msgs.msg import Pose 
from visualization_msgs.msg import Marker
from week4.msg import Group

measured_people = None
current_msg = None
epsilon = .5
group_dist = 4.0

def person_cb(data):
	global measured_people, current_msg

	current_msg = copy.deepcopy(data)
	measured_people = copy.deepcopy(data.people)

# for now I want the same functionality
def identify_groups(people):

	groups = []
	taken_indices = []

	for index, person in enumerate(people):

		if index in taken_indices:
			continue

		n_group = [ person ]
		taken_indices.append(index)

		for j in range(index + 1, len(people)): 

			if j in taken_indices:
				continue

			elif math.dist( [ person.pos.x, person.pos.y ], [ people[j].pos.x, people[j].pos.y ] ) < group_dist:
				n_group.append(people[j])
				taken_indices.append(j)

		groups.append(n_group)

	return groups 

def largest_distance(group):

	# measure distance between people in the group
	distances = []

	for index, person in enumerate(group):

		for j in range(index + 1, len(group)):
			distances.append( [ math.dist( [person.pos.x, person.pos.y], [group[j].pos.x, group[j].pos.y] ), index, j ] )

	# pick the largest as the diameter
	largest_dist_person_1 = -1
	largest_dist_person_2 = -1

	max_dist = -1
	max_ind = -1

	for index, length_pair in enumerate(distances):
		if length_pair[0] > max_dist:
			max_dist = length_pair[0]

			largest_dist_person_1 = length_pair[1]
			largest_dist_person_2 = length_pair[2]

			max_ind = index

	return largest_dist_person_1, largest_dist_person_2, max_dist

def in_a_line(grp):

	xy_poses = []

	current_cp = copy.deepcopy(grp)

	for person in current_cp:
		xy_poses.append([person.pos.x, person.pos.y])

	inline = True

	slope = None

	prev_point = xy_poses[0]
	delta_x = delta_y = 0

	midpoint = copy.deepcopy(xy_poses[0])

	for index, pose in enumerate(xy_poses):

		if index == 0:
			continue

		delta_x = ( pose[0] - prev_point[0] )
		delta_y = ( pose[1] - prev_point[1] )

		n_slope = delta_y / delta_x

		midpoint[0] += pose[0] 
		midpoint[1] += pose[1]

		# print(f'n_slope: {n_slope}, slope: {slope}')

		if slope is not None and abs(n_slope - slope) >= epsilon and inline:
			inline = False

		slope = n_slope

	yaw = math.atan2(delta_y, delta_x)

	midpoint[0] /= len(xy_poses)
	midpoint[1] /= len(xy_poses)

	msg_midpoint = Pose()
	msg_midpoint.position.x = midpoint[0]
	msg_midpoint.position.y = midpoint[1]

	orientation = quaternion_from_euler(0, 0, yaw)

	msg_midpoint.orientation.x = orientation[0]
	msg_midpoint.orientation.y = orientation[1]
	msg_midpoint.orientation.z = orientation[2]
	msg_midpoint.orientation.w = orientation[3]
	

	largest_dist_person_1, largest_dist_person_2, max_dist = largest_distance(current_cp)

	return inline, msg_midpoint, max_dist

def in_a_circle(grp ):

	# take the midpoint as the center of the circle, and calculate the radius
	current_cp = copy.deepcopy(grp)

	largest_dist_person_1, largest_dist_person_2, max_dist = largest_distance(current_cp)

	person_1_point = [ current_cp[largest_dist_person_1].pos.x, current_cp[largest_dist_person_1].pos.y ]
	person_2_point = [ current_cp[largest_dist_person_2].pos.x, current_cp[largest_dist_person_2].pos.y ]

	midpoint = [ ( person_1_point[0] + person_2_point[0] ) / 2 , ( person_1_point[1] + person_2_point[1] ) / 2 ]
	radius = max_dist / 2

	# measure the radius to each other person
	distance_from_center_by_person = []

	for person in current_cp:
		distance_from_center_by_person.append(math.dist(midpoint, [ person.pos.x, person.pos.y ] ) )

	# we can check to see if there are outliers and then reapply the algorithm perhaps? outliers might essentially be checked for if the leg detector simply doesn't see them

	# if the largest distance between the people is too far then they aren't in a circle
	if max_dist > 4.0:
		return False

	msg_midpoint = Pose()
	msg_midpoint.position.x = midpoint[0]
	msg_midpoint.position.y = midpoint[1]

	in_circle = True

	# return true if that radius is within a certain tolerance
	for i in distance_from_center_by_person:
		if abs(i - radius) >= epsilon:
			in_circle = False
			break

	return in_circle, msg_midpoint, radius

def person_measurement_to_person_msg(measurement):
	n_msg = Person()

	n_msg.name = copy.deepcopy(measurement.name)
	n_msg.position = copy.deepcopy(measurement.pos)
	n_msg.reliability = measurement.reliability

	return n_msg

def generate_visualization_msg(group_msg):
	n_msg = Marker()

	split_info = group_msg.group_members[0].name.split('_')

	namespace = split_info[1]

	if group_msg.type == 'circle':
		
		n_msg.header.frame_id = "robot_0/odom";
		n_msg.header.stamp = rospy.Time();
		n_msg.ns = split_info[0] + '_' + namespace
		n_msg.id = int(namespace)

		n_msg.type = 2
		n_msg.action = 0
		n_msg.pose = copy.deepcopy(group_msg.midpoint)

		n_msg.scale.x = group_msg.size * 2
		n_msg.scale.y = group_msg.size * 2
		n_msg.scale.z = .001

		n_msg.color.r = 1
		n_msg.color.a = .34

		n_msg.lifetime.nsecs = 0

		return n_msg

	elif group_msg.type == 'line':

		n_msg.header.frame_id = "robot_0/odom";
		n_msg.header.stamp = rospy.Time();
		n_msg.ns = split_info[0] + '_' + namespace
		n_msg.id = int(namespace)

		n_msg.type = 1
		n_msg.action = 0
		n_msg.pose = copy.deepcopy(group_msg.midpoint)

		n_msg.scale.x = group_msg.size
		n_msg.scale.y = 1.0
		n_msg.scale.z = .001

		n_msg.color.r = 1
		n_msg.color.a = .34

		n_msg.lifetime.nsecs = 0

		return n_msg

	n_msg.header.frame_id = 'robot_0/odom'
	n_msg.header.stamp = rospy.Time()

	n_msg.action = 3

	return n_msg

def main(group_pub, viz_pub):
	global measured_people, current_msgs

	while measured_people == None or current_msg == None:
		pass

	while not rospy.is_shutdown():

		n_msg = Group()

		groups = identify_groups(measured_people)

		midpoint = Pose() 
		size = 0
		group_type = 'other'

		# the underscore is gonna be placed weird in the name if the number of people is larger than 10
		for index, group in enumerate( groups ):
	
			in_line, midpoint_l, size_l = in_a_line(group)
			in_circ, midpoint_c, size_c = in_a_circle(group)

			if len(group) > 1 and in_line:

				n_msg.type = 'line'

				midpoint = midpoint_l
				size = size_l

				for person in group:
					person.name = 'line_' + str(index) + '_' + person.name[:-1] + '_' + person.name[-1]

					n_person = person_measurement_to_person_msg(person)
					n_msg.group_members.append(n_person)

			elif len(group) > 1 and in_circ:

				n_msg.type = 'circle'

				midpoint = midpoint_c
				size = size_c

				for person in group:
					person.name = 'circle_' + str(index) + '_' + person.name[:-1] + '_' + person.name[-1]

					n_person = person_measurement_to_person_msg(person)
					n_msg.group_members.append(n_person)

			else:

				n_msg.type = 'other'

				for person in group:
					person.name = 'other_' + str(index) + '_' + person.name[:-1] + '_' + person.name[-1]

					n_person = person_measurement_to_person_msg(person)
					n_msg.group_members.append(n_person)

			n_msg.size = size
			n_msg.midpoint = midpoint

			viz_msg = generate_visualization_msg(n_msg)

			group_pub.publish(n_msg)
			viz_pub.publish(viz_msg)

		rospy.sleep(.1)

if __name__ == '__main__':

	rospy.init_node('avoider', anonymous=True)

	group_pub = rospy.Publisher('/robot_0/detected_groups', Group, queue_size=10)
	viz_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

	person_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, person_cb)

	main(group_pub, viz_pub)