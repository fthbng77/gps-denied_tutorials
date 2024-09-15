#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from collections import deque

scale_factor_x = -25.3456
scale_factor_y = -42.670277514
scale_factor_z = -32.282871671

# Listeyi son 15 değeri tutacak şekilde sınırlıyoruz
position_history_x = deque(maxlen=15)
position_history_y = deque(maxlen=15)
position_history_z = deque(maxlen=15)

def filter_value(value, history):
    if len(history) < 15:
        return value
    avg = sum(history) / len(history)
    if abs(value - avg) > 2:
        return avg  # Fark 2'den büyükse ortalama değeri kullan
    return value

def transform_pose(orb_pose):
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header
    
    # Konum değerlerini ölçeklendir
    transformed_x = orb_pose.pose.position.x * scale_factor_y
    transformed_y = orb_pose.pose.position.z * scale_factor_x
    transformed_z = orb_pose.pose.position.y * scale_factor_z
    
    # Son gelen veriyi filtrele
    filtered_x = filter_value(transformed_x, position_history_x)
    filtered_y = filter_value(transformed_y, position_history_y)
    filtered_z = filter_value(transformed_z, position_history_z)
    
    # Filtrelenmiş değeri mavros_pose'a ata
    mavros_pose.pose.position.x = filtered_x
    mavros_pose.pose.position.y = filtered_y
    mavros_pose.pose.position.z = filtered_z
    
    mavros_pose.pose.orientation = orb_pose.pose.orientation
    
    # Tarihçeye yeni değeri ekle
    position_history_x.append(transformed_x)
    position_history_y.append(transformed_y)
    position_history_z.append(transformed_z)
    
    return mavros_pose

def pose_callback(data):
    rospy.loginfo("Original ORB-SLAM3 Data: Position: ({}, {}, {})".format(
        data.pose.position.x, data.pose.position.y, data.pose.position.z))
    
    transformed_pose = transform_pose(data)
    
    rospy.loginfo("Transformed Data to MAVROS: Position: ({}, {}, {})".format(
        transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z))
    
    pub.publish(transformed_pose)

rospy.init_node('orb_slam_to_mavros_transform')

sub = rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, pose_callback)
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

rospy.spin()

