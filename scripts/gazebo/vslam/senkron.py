import rospy
from sensor_msgs.msg import Image, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber

def callback(image, imu):
    # IMU ve kamera verilerini işleyin
    rospy.loginfo("Senkronize Mesajlar Alındı")

def main():
    rospy.init_node('sync_node')

    image_sub = Subscriber('/webcam/image_raw', Image)
    imu_sub = Subscriber('/mavros/imu/data', Imu)

    ats = ApproximateTimeSynchronizer([image_sub, imu_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
