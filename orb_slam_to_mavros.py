#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback(data):
    pub.publish(data)

rospy.init_node('orb_slam3_to_mavros')
# ORB-SLAM3 çıktısını doğru topic'ten alacak şekilde güncelleyin
rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, pose_callback)
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

rospy.spin()
