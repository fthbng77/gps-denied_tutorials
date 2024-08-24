#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State

def callback(data):
    rospy.loginfo("Connected: %s, Armed: %s, Mode: %s" % (data.connected, data.armed, data.mode))

def listener():

    rospy.init_node('mavros_state_listener', anonymous=True)

    rospy.Subscriber("mavros/state", State, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
