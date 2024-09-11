#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import math

def vision_pose_publisher():
    # ROS düğümünü başlat
    rospy.init_node('vision_pose_publisher_node', anonymous=True)
    
    # /mavros/vision_pose/pose topiği için bir publisher oluştur
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    
    # Yayın hızını ayarla
    rate = rospy.Rate(4)  # 10 Hz (saniyede 10 defa yayın yapacak)

    # Başlangıç pozisyonu ve hız
    x_position = 0.0
    y_position = 0.0
    z_position = 0.0
    velocity = 2.0  # X ekseninde 2 m/s hız
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        # PoseStamped mesajını oluştur
        pose_msg = PoseStamped()
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        # İlk 12 saniye boyunca sabit pozisyon (x=0, y=0, z=0) yayınla
        if elapsed_time < 12:
            # Header bilgilerini doldur
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"

            # Pozisyonu (0, 0, 0) yap ve varsayılan yönelimi ayarla
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0

            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

        # 12 saniye sonra hareket etmeye başla (1 m/s hızla X ekseninde ilerle)
        else:
            # Header bilgilerini doldur
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"

            # 12 saniyeden sonraki zamanı hesapla ve pozisyonu güncelle
            move_time = elapsed_time - 12  # Hareketin başladığı andan itibaren geçen süre
            x_position = velocity * move_time

            # Güncellenmiş pozisyonu ayarla ve varsayılan yönelimi koru
            pose_msg.pose.position.x = x_position
            pose_msg.pose.position.y = y_position
            pose_msg.pose.position.z = z_position

            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

        # Mesajı yayınla
        pose_pub.publish(pose_msg)

        # Yayın hızını koru
        rate.sleep()

if __name__ == '__main__':
    try:
        vision_pose_publisher()
    except rospy.ROSInterruptException:
        pass

