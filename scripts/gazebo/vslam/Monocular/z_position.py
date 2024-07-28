#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

scale_factor = 27.7 * (1.2)

def transform_pose(orb_pose):
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header

    # Sadece z pozisyonunu ölçeklendiriyoruz
    mavros_pose.pose.position.z = orb_pose.pose.position.z * scale_factor
    
    # x ve y pozisyonlarını sabit tut
    mavros_pose.pose.position.x = 0
    mavros_pose.pose.position.y = 0
    
    # Orientation'u sabit tutarak stabiliteyi sağlıyoruz
    mavros_pose.pose.orientation.x = 0
    mavros_pose.pose.orientation.y = 0
    mavros_pose.pose.orientation.z = 0
    mavros_pose.pose.orientation.w = 1
    
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
