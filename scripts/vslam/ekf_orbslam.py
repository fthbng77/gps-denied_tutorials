#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from filterpy.kalman import KalmanFilter
from scipy.signal import butter, lfilter
import numpy as np

scale_factor = 44.32  # Önceden belirlenen veya deneysel olarak bulunan ölçek faktörü

fs = 100.0       # Örnekleme frekansı (Hz)
cutoff = 3.0     # Kesim frekansı (Hz)
b, a = butter(1, cutoff / (0.5 * fs), btype='low')

previous_filtered_x = 0
previous_filtered_y = 0
previous_filtered_z = 0

# Kalman filtresi oluştur
kf = KalmanFilter(dim_x=6, dim_z=3)
kf.x = np.zeros(6)  # Durum vektörü (x, y, z, vx, vy, vz)
kf.F = np.eye(6)  # Durum geçiş matrisi
kf.H = np.zeros((3, 6))  # Gözlem matrisi
kf.H[:3, :3] = np.eye(3)
kf.P *= 1000. 
kf.R = np.eye(3) * 5 
kf.Q = np.eye(6) * 0.01 

def transform_pose(orb_pose):
    # ORB-SLAM3'ten MAVROS'a koordinat dönüşümü
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header
    mavros_pose.pose.position.x = orb_pose.pose.position.z * scale_factor
    mavros_pose.pose.position.y = -orb_pose.pose.position.x * scale_factor
    mavros_pose.pose.position.z = -orb_pose.pose.position.y * scale_factor
    
    return mavros_pose

def pose_callback(data):
    global previous_filtered_x, previous_filtered_y, previous_filtered_z

    # Gelen verileri düşük geçiren filtreden geçir
    current_x = lfilter(b, a, [previous_filtered_x, data.pose.position.x])[-1]
    current_y = lfilter(b, a, [previous_filtered_y, data.pose.position.y])[-1]
    current_z = lfilter(b, a, [previous_filtered_z, data.pose.position.z])[-1]

    # Önceki verileri güncelle
    previous_filtered_x, previous_filtered_y, previous_filtered_z = current_x, current_y, current_z

    rospy.loginfo("Filtered ORB-SLAM3 Data: Position: ({}, {}, {})".format(current_x, current_y, current_z))

    transformed_pose = transform_pose(data)
    z = np.array([transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z])
    kf.predict()
    kf.update(z)
    filtered_position = kf.x[:3]

    rospy.loginfo("Transformed Data to MAVROS (Filtered): Position: ({}, {}, {})".format(
        filtered_position[0], filtered_position[1], filtered_position[2]))

    filtered_pose = PoseStamped()
    filtered_pose.header = transformed_pose.header
    filtered_pose.pose.position.x = filtered_position[0]
    filtered_pose.pose.position.y = filtered_position[1]
    filtered_pose.pose.position.z = filtered_position[2]

    pub.publish(filtered_pose)

rospy.init_node('orb_slam_to_mavros_transform')

sub = rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, pose_callback)
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

rospy.spin()