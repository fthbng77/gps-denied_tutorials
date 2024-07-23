#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

scale_factor = 44.32

# Global değişkenler
orb_pose = None
imu_orientation = None

# Düşük geçişli filtre sınıfı
class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.prev_val = None

    def apply(self, value):
        if self.prev_val is None:
            self.prev_val = value
        filtered_val = self.alpha * value + (1 - self.alpha) * self.prev_val
        self.prev_val = filtered_val
        return filtered_val

# Filtreler için global değişkenler
roll_filter = LowPassFilter()
pitch_filter = LowPassFilter()
yaw_filter = LowPassFilter()

def transform_pose(orb_pose):
    # ORB-SLAM3'ten MAVROS'a koordinat dönüşümü
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header
    mavros_pose.pose.position.x = orb_pose.pose.position.z * scale_factor
    mavros_pose.pose.position.y = -orb_pose.pose.position.x * scale_factor
    mavros_pose.pose.position.z = -orb_pose.pose.position.y * scale_factor
    
    return mavros_pose

def pose_callback(data):
    global orb_pose
    orb_pose = data
    publish_combined_pose()

def imu_callback(data):
    global imu_orientation
    imu_orientation = data.orientation
    orientation_list = [imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
    # Filtre uygulanmış veriler
    filtered_roll = roll_filter.apply(roll)
    filtered_pitch = pitch_filter.apply(pitch)
    filtered_yaw = yaw_filter.apply(yaw)
    
    rospy.loginfo("Filtered IMU Data: Roll: {:.4f}, Pitch: {:.4f}, Yaw: {:.4f}".format(filtered_roll, filtered_pitch, filtered_yaw))
    
    publish_combined_pose()

def publish_combined_pose():
    global orb_pose, imu_orientation
    if orb_pose is not None and imu_orientation is not None:
        transformed_pose = transform_pose(orb_pose)
        
        # Filtrelenmiş IMU verilerini kullanarak yeni bir quaternion hesaplama
        (roll, pitch, yaw) = euler_from_quaternion([imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w])
        filtered_roll = roll_filter.apply(roll)
        filtered_pitch = pitch_filter.apply(pitch)
        filtered_yaw = yaw_filter.apply(yaw)
        filtered_orientation = quaternion_from_euler(filtered_roll, filtered_pitch, filtered_yaw)
        
        transformed_pose.pose.orientation.x = filtered_orientation[0]
        transformed_pose.pose.orientation.y = filtered_orientation[1]
        transformed_pose.pose.orientation.z = filtered_orientation[2]
        transformed_pose.pose.orientation.w = filtered_orientation[3]
        
        rospy.loginfo("Publishing Combined Data: Position: ({}, {}, {}), Orientation: ({}, {}, {}, {}))".format(
            transformed_pose.pose.position.x, transformed_pose.pose.position.y, 
            transformed_pose.pose.position.z, transformed_pose.pose.orientation.x, 
            transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, 
            transformed_pose.pose.orientation.w))
        pub.publish(transformed_pose)

def listener():
    rospy.init_node('combined_orb_slam_imu', anonymous=True)

    # Pose ve IMU için subscriber'lar
    rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, pose_callback)
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)

    # Pose dönüşümü için publisher
    global pub
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
