#!/usr/bin/python3

import rospy
import copy
import math
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Point 
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
		xy_poses.append([person.pos.x, person.msg_pos.y])

	inline = True

	slope = None

	prev_point = xy_poses[0]

	midpoint = copy.deepcopy(xy_poses[0])

	for index, pose in enumerate(xy_poses):

		if index == 0:
			continue

		n_slope = ( pose[1] - prev_point[1] ) / ( pose[0] - prev_point[0] )

		midpoint[0] += pose[0] 
		midpoint[1] += pose[1]

		# print(f'n_slope: {n_slope}, slope: {slope}')

		if slope is not None and abs(n_slope - slope) >= epsilon and inline:
			inline = False

		slope = n_slope

	midpoint[0] /= len(xy_poses)
	midpoint[1] /= len(xy_poses)

	msg_midpoint = Pose()
	msg_midpoint.position.x = midpoint[0]
	msg_midpoint.position.y = midpoint[1]

	largest_dist_person_1, largest_dist_person_2, max_dist = largest_distance(current_cp)

	return inline, msg_midpoint, max_dist

def in_a_circle(grp ):

	# take the midpoint as the center of the circle, and calculate the radius

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
	pass

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
	
			in_a_line, midpoint_l, size_l = in_a_line(group)
			in_a_circ, midpoint_c, size_c = in_a_circle(group)

			if len(group) > 1 and in_a_line:

				n_msg.type = 'line'

				midpoint = midpoint_l
				size = size_l

				for person in group:
					person.name = 'line_' + str(index) + '_' + person.name[:-1] + '_' + person.name[-1]

					n_person = person_measurement_to_person_msg(person)
					n_msg.group_members.append(n_person)

			elif len(group) > 1 and in_a_circ:

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

		viz_msg = generate_visualization_msg(n_msg, midpoint, size)

		n_msg.size = size
		n_msg.midpoint = midpoint

		group_pub.publish(n_msg)
		viz_pub.publish(viz_msg)

		rospy.sleep(.1)

if __name__ == '__main__':

	rospy.init_node('avoider', anonymous=True)

	group_pub = rospy.Publisher('/robot_0/detected_groups', Group, queue_size=10)


	person_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, person_cb)

	main(group_pub)