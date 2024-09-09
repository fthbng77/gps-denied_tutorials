#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber

def callback(image_data, imu_data):
    # Kamera ve IMU verilerini bu fonksiyonda senkronize olarak işleyebilirsiniz
    rospy.loginfo("Synchronized data received!")
    rospy.loginfo("Image received with timestamp: {}".format(image_data.header.stamp))
    rospy.loginfo("IMU data: Angular velocity ({}, {}, {})".format(imu_data.angular_velocity.x,
                                                                   imu_data.angular_velocity.y,
                                                                   imu_data.angular_velocity.z))

# ROS node'u başlat
rospy.init_node('sync_node')

# Kamera ve IMU verileri için Subscriber oluştur
image_sub = Subscriber('/webcam/image_raw', Image)
imu_sub = Subscriber('/mavros/imu/data', Imu)

# Yaklaşık zaman senkronizasyonu için ApproximateTimeSynchronizer kullan
ats = ApproximateTimeSynchronizer([image_sub, imu_sub], queue_size=10, slop=0.1)
ats.registerCallback(callback)

# Sonsuz döngü
rospy.spin()
