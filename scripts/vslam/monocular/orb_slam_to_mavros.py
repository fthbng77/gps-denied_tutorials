#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

scale_factor_x = -25.3456
scale_factor_y = -42.670277514
scale_factor_z =  32.282871671
def transform_pose(orb_pose):
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header
    mavros_pose.pose.position.x = orb_pose.pose.position.x * scale_factor_y
    mavros_pose.pose.position.y = orb_pose.pose.position.z * scale_factor_x
    mavros_pose.pose.position.z = orb_pose.pose.position.y  * scale_factor_z
    
    mavros_pose.pose.orientation = orb_pose.pose.orientation
    return mavros_pose

def pose_callback(data):
    rospy.loginfo("Original ORB-SLAM3 Data: Position: ({}, {}, {}))".format(
        data.pose.position.x, data.pose.position.y, data.pose.position.z))
        
    transformed_pose = transform_pose(data)
    rospy.loginfo("Transformed Data to MAVROS: Position: ({}, {}, {}))".format(
        transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z))
    pub.publish(transformed_pose)

rospy.init_node('orb_slam_to_mavros_transform')

sub = rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, pose_callback)
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

rospy.spin()