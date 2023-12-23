#!/usr/bin/env python3

import rospy
import copy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import threading
import matplotlib.pyplot as plt
import os.path as path 
import sys
import numpy as np
from PIL import Image
from sensor_msgs.msg import LaserScan
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement

my_pos = None
laser_data = None
most_likely_person = None

my_map = []
image_size = [ 35, 35, .5 ]
meter_to_pixel = 1 / 33
center_coords = []

def set_pos_cb(data):
    global my_pos 

    my_pos = data.pose.pose

def laser_cb(data):
    global laser_data

    laser_data = data

def person_cb(data):
    global most_likely_person

    max_rel = -1
    current_person = None

    for i in data.people:
        if i.reliability > max_rel and that_person_is_not_in_a_wall(i):
            max_rel = i.reliability
            current_person = copy.deepcopy(i)

    most_likely_person = current_person

def read_map(f):
    global center_coords, my_map 
    my_map = Image.open(f)

    my_map = my_map.rotate(90)

    my_map = np.asarray(my_map)

    print(my_map)

    center_coords = [ int(len(my_map) / 2), int(len(my_map[0]) / 2) ]

    print(my_map[center_coords[0]][center_coords[1]])

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


def find_closest_obstacle():    
    min_range = 10000000 

    for i in laser_data.ranges:
        if i < min_range:
            min_range = i

    return min_range

def find_longest_sequence(seq_num):
    global laser_data

    longest_num = -1
    longest_seq_index = -1
    in_seq = False

    seq_min = -1
    seq_len = -1

    seqs = []

    for i in range(len(laser_data.ranges)):
        if laser_data.ranges[i] == seq_num:
            if not in_seq:
                seq_min = i
                seq_len = 0

                in_seq = True
            
            seq_len += 1

        else:
            if in_seq:
                seqs.append([seq_min, i])

                if seq_len > longest_num:
                    longest_num = seq_len
                    longest_seq_index = len(seqs) - 1

            in_seq = False
            seq_min = -1
            seq_len = -1
            
    if in_seq:
        seqs.append([seq_min, len(laser_data.ranges) - 1])

        if seq_len > longest_num:
            longest_num = seq_len
            longest_seq_index = len(seqs) - 1

    # print(f'biggest index is {longest_seq_index}')
    # for i in seqs:
    #     print(f'seq {i[0]} -> {i[1]} of len {i[1] - i[0]}')

    return seqs[longest_seq_index]

def find_farthest_obstacle_angle():
    current_max = -1

    for i in range(len(laser_data.ranges)):
        if laser_data.ranges[i] > current_max:
            current_max = laser_data.ranges[i]

    longest_seq = find_longest_sequence(current_max)

    return laser_data.angle_min + ( math.floor( ( longest_seq[1] - longest_seq[0] ) / 2 ) + longest_seq[0] ) * laser_data.angle_increment 

def go_forward_until_obstacle(pub, min_dist):

    global laser_data

    twistmsg = Twist()

    print("going forward!")

    while my_pos == None and laser_data == None:
        pass

    try:
        twistmsg.linear.x = 1.0

        # until we've moved the specified square side length
        while find_closest_obstacle() >= min_dist:

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

def get_smallest_dist_and_direction(ang1, ang2):

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
        return distance2, 'left'
    else:
        return distance1, 'right'

def turn_right(pub, rad, forward_speed):
    twistmsg = Twist()

    print("turning right!")

    try:
        (init_r, init_p, init_y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ])

        end_y = init_y - rad

        cur_y = init_y

        if end_y >= math.pi:
            end_y += math.pi * 2

        twistmsg.angular.z = -1 * math.pi / 8
        twistmsg.linear.x = forward_speed


        # was lazy so found smallest angle approach
        while not is_same_angle(cur_y, end_y, .08):

            (r, p, y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ] )

            cur_y = y

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

def turn_toward(pub, angle):
    (init_r, init_p, init_y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ])

    dist, direction = get_smallest_dist_and_direction(init_y, angle)

    if 'right' == direction:
        turn_right(pub, dist, 0.0)
    else:
        turn_left(pub, dist, 0.0)
    
def transform_sensor_angle_to_world_angle(angle):
    global my_pos

    (init_r, init_p, init_y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ])

    return init_y + angle

# move in directions with the least number of objects
def run_avoidance(pub):
    global my_pos, laser_data

    while my_pos == None or laser_data == None:
        pass

    while not rospy.is_shutdown():
       go_forward_until_obstacle(pub, 1.0)

       angle = find_farthest_obstacle_angle()

       world_angle = transform_sensor_angle_to_world_angle(angle)

       turn_toward(pub, world_angle)

       go_forward(pub, .4)

def avoid_obstacle(pub):
   global my_pos, laser_data

   angle = find_farthest_obstacle_angle()

   world_angle = transform_sensor_angle_to_world_angle(angle)

   turn_toward(pub, world_angle)

   go_forward(pub, .6)

def detect_closest_person():
    global most_likely_person, my_pos

    if most_likely_person == None:
        return -1, -1

    angle_between = math.atan2(most_likely_person.pos.y - my_pos.position.y, most_likely_person.pos.x - my_pos.position.x)

    if angle_between < 0:
        angle_between += 2 * math.pi

    return [most_likely_person.pos.x, most_likely_person.pos.y], angle_between

def get_laser_pos_in_odom(laser_dist, laser_angle):
    pass

def get_person_angle_in_laser(person):
    pass

def that_person_is_not_in_a_wall(person):
    global my_pos, laser_data

    # first check if there is a person in the scene

    return True

def follow_people(pub):
    global my_pos, laser_data, most_likely_person

    while my_pos == None or laser_data == None:
        pass

    goal_loc = None
    goal_angle = -10

    while not rospy.is_shutdown():
        loc, angle = detect_closest_person()

        if goal_loc == None or goal_angle == -10:

            if ( loc == -1 and angle == -1 ):
                print('can\'t find person turning right 90 degrees')

                turn_right(pub, math.pi / 2, 0.0)

                rospy.sleep(2)

                goal_loc = None
                goal_angle = -10

                continue

            goal_loc = loc
            goal_angle = angle

            print(f'Person detected at {goal_loc}! Starting pursuit...')


        turn_toward(pub, goal_angle)

        go_forward_until_obstacle(pub, .8)

        if math.dist([ my_pos.position.x, my_pos.position.y ] , goal_loc ) < 2.0:
            print(f"I found them at {goal_loc}! Please hit enter for me to follow them again...", end='')

            input()

            goal_loc = None
            goal_angle = -10

            continue

        avoid_obstacle(pub)

        if math.dist([ my_pos.position.x, my_pos.position.y ] , goal_loc ) < 1.0:
            print(f"I found them at {goal_loc}! Please hit enter for me to follow them again...", end='')

            input()

            goal_loc = None
            goal_angle = -10

if __name__ == '__main__':


    rospy.init_node('avoider', anonymous=True)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    laser_sub = rospy.Subscriber("base_scan", LaserScan, laser_cb)

    odom_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, set_pos_cb)

    person_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, person_cb)

    map_loc = __file__

    head, tail = path.split(map_loc)

    map_loc, tail = path.split(head)

    map_loc += '/world/basic_map.pgm'

    # read_map(map_loc)

    follow_people(pub)