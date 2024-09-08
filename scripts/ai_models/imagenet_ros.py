#!/usr/bin/env python3
from jetson_inference import imageNet
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from jetson_utils import cudaFromNumpy
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    global cv_img
    try:
        # ROS görüntü mesajını OpenCV görüntüsüne dönüştür
        cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_img = cv_img.copy()  # Görüntünün yazılabilir olduğundan emin ol

        cuda_img = cudaFromNumpy(cv_img)

        # Görüntüyü sınıflandır
        class_idx, confidence = net.Classify(cuda_img)
        class_desc = net.GetClassDesc(class_idx)
        pub.publish(class_desc)

        # Görüntü üzerinde sonucu göster
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = "{}: {:.2f}%".format(class_desc, confidence * 100)
        cv2.putText(cv_img, text, (10, 30), font, 1, (255, 255, 255), 2)

        rospy.loginfo("Görüntü '{:s}' olarak tanındı (sınıf #{:d}) %{:f} güvenle".format(class_desc, class_idx, confidence * 100))
    except Exception as e:
        rospy.logerr("Görüntü callback'inde hata: {}".format(e))

def main():
    global net, pub, cv_img

    net = imageNet("resnet-18")

    rospy.init_node('imagenet_node', anonymous=True)

    sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    pub = rospy.Publisher('/imagenet_result', String, queue_size=10)

    # Görüntüyü göstermek için bir pencere oluştur
    cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Image", 640, 480)
    
    while not rospy.is_shutdown():
        if cv_img is not None:
            # Görüntüyü göster
            cv2.imshow("Image", cv_img)
        cv2.waitKey(1)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    cv_img = None
    main()

