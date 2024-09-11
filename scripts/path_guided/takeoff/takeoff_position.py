#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def publish_zero_pose():
    rospy.init_node('straight_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(4)

    # Başlangıç zamanı
    start_time = rospy.get_time()

    # Kare çizme süresi ve hız
    takeoff = 5.73
    vel = 0.8726 #hız

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed_time = current_time - start_time

        # PoseStamped mesajını oluştur
        pose_msg = PoseStamped()

        # Header bilgisi (timestamp ve frame id)
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"  # Frame ID'yi ihtiyaçlarına göre değiştirebilirsin

        # TAKEOFF
        if elapsed_time < 10:
            # Pozisyon bilgisi sıfır
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0

        # İLERİ
        elif elapsed_time < takeoff + 10:
            pose_msg.pose.position.x = 0 #hareketin olduğu eksen
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = vel * (elapsed_time - 8)

        # DUR
        else:
            pose_msg.pose.position.x = 0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 5

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

