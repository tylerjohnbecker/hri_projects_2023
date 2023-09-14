#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from week0.msg import Test
from sensor_msgs.msg import LaserScan

def callback(data):

    min_range = 10000000 

    for i in data.ranges:
        if i < min_range:
            min_range = i

    rospy.loginfo(f'Minimum laser distance: {min_range}')
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('scan_listener', anonymous=True)

    rospy.Subscriber("base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()