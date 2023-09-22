#!/usr/bin/env python3

import rospy
import copy
import geometry_msgs
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import threading
import tf2_ros
from sensor_msgs.msg import LaserScan
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement

global_broadcaster = None
my_pos = None

def set_pos_cb(data):
    global my_pos 

    my_pos = data.pose.pose


def receive_and_echo_cb (data):
    global global_broadcaster
    
    while my_pos == None:
        pass

    t = geometry_msgs.msg.TransformStamped()

    mrp = None
    highest_reliability = -1

    for i in data.people:
        if i.reliability < highest_reliability:
            highest_reliability = i.reliability
            mrp = i

    if mrp == None:
        return

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = 'leg_1'
    t.transform.translation.x = my_pos.position.x + mrp.pos.x
    t.transform.translation.y = my_pos.position.y + mrp.pos.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    global_broadcaster.sendTransform(t)

if __name__ == '__main__':

    rospy.init_node('avoider', anonymous=True)

    # global_pub = rospy.Publisher('leg_tracker_measurements', PositionMeasurementArray, queue_size=10)

    person_sub = rospy.Subscriber('leg_tracker_measurements', PositionMeasurementArray, receive_and_echo_cb)

    global_broadcaster = tf2_ros.TransformBroadcaster()

    odom_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, set_pos_cb)

    rospy.spin()