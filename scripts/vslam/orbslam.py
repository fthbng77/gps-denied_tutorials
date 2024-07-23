#!/usr/bin/env python
"""
Görsel tabanlı sonuçlar doğrultusunda düzenlenen 
"""

import rospy
from geometry_msgs.msg import PoseStamped
from transformations import quaternion_multiply, quaternion_about_axis, quaternion_conjugate

import numpy as np

# Ölçek faktörünü belirle
scale_factor = 44.32  # Önceden belirlenen veya deneysel olarak bulunan ölçek faktörü

def transform_pose(orb_pose):
    # ORB-SLAM3'ten MAVROS'a koordinat dönüşümü
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header
    mavros_pose.pose.position.x = orb_pose.pose.position.z * scale_factor
    mavros_pose.pose.position.y = -orb_pose.pose.position.x * scale_factor
    mavros_pose.pose.position.z = -orb_pose.pose.position.y * scale_factor
    
    # ORB-SLAM3 orientasyonundan MAVROS orientasyonuna dönüşüm
    quaternion = (
        orb_pose.pose.orientation.x,
        orb_pose.pose.orientation.y,
        orb_pose.pose.orientation.z,
        orb_pose.pose.orientation.w
    )
    # Z ekseni etrafında 180 derece döndür
    z_rotation = quaternion_about_axis(np.pi, (0, 0, 1))
    quaternion_rotated = quaternion_multiply(quaternion, z_rotation)
    quaternion_corrected = quaternion_conjugate(quaternion_rotated)

    mavros_pose.pose.orientation.x = quaternion_rotated[0]
    mavros_pose.pose.orientation.y = quaternion_rotated[1]
    mavros_pose.pose.orientation.z = quaternion_rotated[2]
    mavros_pose.pose.orientation.w = quaternion_rotated[3]
    
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
