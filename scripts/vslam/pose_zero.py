#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def publish_zero_pose():
    # ROS node başlat
    rospy.init_node('zero_pose_publisher', anonymous=True)

    # PoseStamped mesajını mavros/vision_pose/pose topici üzerinden yayınlayan bir publisher oluştur
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

    # Yayınlama frekansı (10 Hz olarak ayarlıyoruz)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # PoseStamped mesajını oluştur
        pose_msg = PoseStamped()

        # Header bilgisi (timestamp ve frame id)
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"  # Frame ID'yi ihtiyaçlarına göre değiştirebilirsin

        # Pozisyon bilgisi sıfır
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0

        # Yönelim bilgisi (quaternion olarak; burada da sıfır olarak yayınlanıyor)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        # Mesajı yayınla
        pose_pub.publish(pose_msg)

        # Bekle ve tekrar yayınla
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_zero_pose()
    except rospy.ROSInterruptException:
        pass

