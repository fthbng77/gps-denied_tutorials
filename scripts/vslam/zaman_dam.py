import rospy
from sensor_msgs.msg import Image, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rospy import Publisher

def callback(image, imu, image_pub, imu_pub):
    # IMU ve kamera verilerini işleyin
    rospy.loginfo("Senkronize Mesajlar Alındı")

    # Senkronize verileri yeniden yayınla
    image_pub.publish(image)
    imu_pub.publish(imu)

def main():
    rospy.init_node('sync_node')

    image_sub = Subscriber('/webcam/image_raw', Image)
    imu_sub = Subscriber('/mavros/imu/data', Imu)

    image_pub = Publisher('/sync_node/image_raw', Image, queue_size=10)
    imu_pub = Publisher('/sync_node/imu/data', Imu, queue_size=10)

    ats = ApproximateTimeSynchronizer([image_sub, imu_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback, image_pub, imu_pub)

    rospy.spin()

if __name__ == '__main__':
    main()
