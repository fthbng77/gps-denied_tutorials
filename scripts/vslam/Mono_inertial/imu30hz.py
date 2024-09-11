#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, Image
from scipy.interpolate import interp1d
import numpy as np

# IMU zaman damgalarını ve verilerini tutmak için listeler
imu_times = []
imu_angular_velocities = []
imu_linear_accelerations = []

# IMU verilerini toplama fonksiyonu
def imu_callback(data):
    time_stamp = data.header.stamp.to_sec()
    angular_velocity = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
    linear_acceleration = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]

    imu_times.append(time_stamp)
    imu_angular_velocities.append(angular_velocity)
    imu_linear_accelerations.append(linear_acceleration)

# IMU verilerini interpolasyonla işleyip yeni bir konuya yayınlama fonksiyonu
def image_callback(image_data):
    image_time = image_data.header.stamp.to_sec()

    # Hedef zaman damgasını (kamera verisinin zaman damgası) belirle
    target_timestamp = np.array([image_time])

    # IMU verilerini interpolasyonla bu zaman damgasına uydur
    interpolated_angular, interpolated_linear = interpolate_imu_data(target_timestamp)

    # Yeni IMU mesajı oluştur
    synced_imu_msg = Imu()
    synced_imu_msg.header.stamp = rospy.Time.now()  # Şu anki zaman damgasını kullanıyoruz
    synced_imu_msg.angular_velocity.x = interpolated_angular[0][0]
    synced_imu_msg.angular_velocity.y = interpolated_angular[0][1]
    synced_imu_msg.angular_velocity.z = interpolated_angular[0][2]
    synced_imu_msg.linear_acceleration.x = interpolated_linear[0][0]
    synced_imu_msg.linear_acceleration.y = interpolated_linear[0][1]
    synced_imu_msg.linear_acceleration.z = interpolated_linear[0][2]

    # IMU verisini yeni bir ROS konusu altında yayınla
    synced_imu_pub.publish(synced_imu_msg)

    rospy.loginfo("Published synced IMU data at 30 Hz")

# IMU verileri için interpolasyon fonksiyonu
def interpolate_imu_data(target_timestamps):
    imu_times_np = np.array(imu_times)
    imu_angular_np = np.array(imu_angular_velocities)
    imu_linear_np = np.array(imu_linear_accelerations)

    angular_interp = interp1d(imu_times_np, imu_angular_np, axis=0, kind='linear', fill_value="extrapolate")
    linear_interp = interp1d(imu_times_np, imu_linear_np, axis=0, kind='linear', fill_value="extrapolate")

    interpolated_angular = angular_interp(target_timestamps)
    interpolated_linear = linear_interp(target_timestamps)

    return interpolated_angular, interpolated_linear

# ROS node'u başlat
rospy.init_node('imu_interpolator')

# Senkronize IMU verileri için publisher oluştur
synced_imu_pub = rospy.Publisher('/synced_imu/data', Imu, queue_size=10)

# Kamera ve IMU verileri için Subscriber'lar oluştur
sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
sub_image = rospy.Subscriber('/webcam/image_raw', Image, image_callback)

# Sonsuz döngü
rospy.spin()

