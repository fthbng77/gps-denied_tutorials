#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

scale_factor = 27.7
alpha = 0.5  # Düşük geçiş filtresi için daha agresif bir alfa değeri

previous_x = 0
previous_y = 0

def low_pass_filter(new_value, old_value, alpha):
    return alpha * new_value + (1 - alpha) * old_value

def transform_pose(orb_pose):
    global previous_x, previous_y

    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header

    # z pozisyonunu doğrudan ölçeklendiriyoruz
    z_position = orb_pose.pose.position.z * scale_factor

    # x ve y pozisyonlarını ölçeklendiriyoruz
    x_position = orb_pose.pose.position.z * scale_factor
    y_position = -orb_pose.pose.position.x * scale_factor

    # Düşük geçiş filtresi uygulayarak x ve y pozisyonlarını yumuşatıyoruz
    filtered_x = low_pass_filter(x_position, previous_x, alpha)
    filtered_y = low_pass_filter(y_position, previous_y, alpha)

    previous_x = filtered_x
    previous_y = filtered_y

    mavros_pose.pose.position.x = filtered_x
    mavros_pose.pose.position.y = filtered_y
    mavros_pose.pose.position.z = z_position
    
    # Orientation'u doğrudan kopyalıyoruz, çünkü compass'tan geliyor
    mavros_pose.pose.orientation = orb_pose.pose.orientation
    
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