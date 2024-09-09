#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

# IMU verilerini toplama ve doğrudan yayınlama fonksiyonu
def imu_callback(data):
    # IMU verisini doğrudan 200 Hz'de yayınla
    raw_imu_pub.publish(data)

    rospy.loginfo("Published raw IMU data at 200 Hz")

# ROS node'u başlat
rospy.init_node('imu_publisher_200hz')

# 200 Hz IMU verisi için publisher oluştur
raw_imu_pub = rospy.Publisher('/raw_imu/data', Imu, queue_size=10)

# IMU verisi için subscriber oluştur
sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)

# Rate'i 200 Hz olarak ayarlayın
rate = rospy.Rate(200)

# Sonsuz döngü
while not rospy.is_shutdown():
    rate.sleep()

