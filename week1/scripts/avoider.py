#!/usr/bin/env python3

import rospy
import copy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import threading
from sensor_msgs.msg import LaserScan

my_pos = None
laser_data = None

def set_pos_cb(data):
    global my_pos 

    my_pos = data.pose.pose

def laser_cb(data):
    global laser_data

    laser_data = data

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

    print(f'biggest index is {longest_seq_index}')
    for i in seqs:
        print(f'seq {i[0]} -> {i[1]} of len {i[1] - i[0]}')

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

        print(f'found an obstacle {find_closest_obstacle()} away!')

    
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

def turn_toward(pub, angle):
    (init_r, init_p, init_y) = euler_from_quaternion([ my_pos.orientation.x, my_pos.orientation.y, my_pos.orientation.z, my_pos.orientation.w ])

    dist, direction = get_smallest_dist_and_direction(init_y, angle)

    print(f'going {direction} this amount {dist} from {init_y} to end up at {angle}')

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

       print(f'found object at angle {angle}')

       world_angle = transform_sensor_angle_to_world_angle(angle)

       print(f'angle{world_angle}')

       turn_toward(pub, world_angle)

       go_forward(pub, .4)



if __name__ == '__main__':


    rospy.init_node('traveler', anonymous=True)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    laser_sub = rospy.Subscriber("base_scan", LaserScan, laser_cb)

    odom_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, set_pos_cb)

    run_avoidance(pub)