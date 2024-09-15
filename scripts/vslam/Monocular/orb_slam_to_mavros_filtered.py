#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from collections import deque

# Scale factors
scale_factor_x = -28.670277514
scale_factor_y = -42.670277514
scale_factor_z = -32.282871671

# History for storing position values with a larger window size
position_history_x = deque(maxlen=30)  # Increased window for smoother results
position_history_y = deque(maxlen=30)
position_history_z = deque(maxlen=30)

def filter_value(value, history):
    history.append(value)
    
    # Apply moving average filter
    if len(history) < 5:  # Need at least 5 values to calculate a reliable average
        return value
    
    # Calculate the moving average over the window (you can increase the window for more smoothing)
    return sum(history) / len(history)

def transform_pose(orb_pose):
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header
    
    # Scale the position values
    transformed_x = orb_pose.pose.position.x * scale_factor_y
    transformed_y = orb_pose.pose.position.z * scale_factor_x
    transformed_z = orb_pose.pose.position.y * scale_factor_z
    
    # Apply filtering to each axis
    filtered_x = filter_value(transformed_x, position_history_x)
    filtered_y = filter_value(transformed_y, position_history_y)
    filtered_z = filter_value(transformed_z, position_history_z)
    
    # Assign the filtered values to the pose
    mavros_pose.pose.position.x = filtered_x
    mavros_pose.pose.position.y = filtered_y
    mavros_pose.pose.position.z = filtered_z
    
    mavros_pose.pose.orientation = orb_pose.pose.orientation
    
    return mavros_pose

def pose_callback(data):
    rospy.loginfo("Original ORB-SLAM3 Data: Position: ({}, {}, {})".format(
        data.pose.position.x, data.pose.position.y, data.pose.position.z))
    
    transformed_pose = transform_pose(data)
    
    rospy.loginfo("Transformed Data to MAVROS: Position: ({}, {}, {})".format(
        transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z))
    
    pub.publish(transformed_pose)

# ROS node initialization
rospy.init_node('orb_slam_to_mavros_transform')

sub = rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, pose_callback)
pub = rospy.Publisher('/mavros/vision_pose/pose_deneme', PoseStamped, queue_size=10)

rospy.spin()
