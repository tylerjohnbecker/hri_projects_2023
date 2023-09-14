#!/usr/bin/env python3
# license removed for brevity
import rospy
from week0.msg import Test

def talker():
    pub = rospy.Publisher('chatter', Test, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    msg = Test()
    msg.x = 10.0
    msg.y = 12.9
    msg.z = 12

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass