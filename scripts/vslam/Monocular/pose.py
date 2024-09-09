#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft

# Ölçek faktörünü belirle
scale_factor = 44.32  # Önceden belirlenen veya deneysel olarak bulunan ölçek faktörü

def transform_pose(orb_pose):
    # ORB-SLAM3'ten MAVROS'a koordinat dönüşümü
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header
    
    #mavros_pose.pose.position.x = orb_pose.pose.position.z * scale_factor
    mavros_pose.pose.position.x = -orb_pose.pose.position.x * scale_factor
    #mavros_pose.pose.position.y = -orb_pose.pose.position.x * scale_factor
    mavros_pose.pose.position.y = orb_pose.pose.position.y * scale_factor
    #mavros_pose.pose.position.z = -orb_pose.pose.position.y * scale_factor
    mavros_pose.pose.position.z = orb_pose.pose.position.z * scale_factor
    
    # Quaternion'ı Euler açılarına çevir
    orb_orientation = orb_pose.pose.orientation
    (roll, pitch, yaw) = tft.euler_from_quaternion([orb_orientation.x, orb_orientation.y, orb_orientation.z, orb_orientation.w])
    
    # Eğer gerekirse roll, pitch, yaw değerlerini değiştirin
    # Burada basitçe x, y, z eksenleriyle oryantasyonun eşleştiğini varsayıyoruz
    transformed_quaternion = tft.quaternion_from_euler(roll, -yaw, -pitch)
    
    # MAVROS için orientasyon dönüşümü
    mavros_pose.pose.orientation.x = transformed_quaternion[0]
    mavros_pose.pose.orientation.y = transformed_quaternion[1]
    mavros_pose.pose.orientation.z = transformed_quaternion[2]
    mavros_pose.pose.orientation.w = transformed_quaternion[3]
    
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
