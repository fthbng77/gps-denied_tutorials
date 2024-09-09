#!/usr/bin/env python3
from jetson_inference import detectNet
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from jetson_utils import cudaFromNumpy
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    global cv_img
    cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    cuda_img = cudaFromNumpy(cv_img)

    # Detect the objects in the image
    detections = net.Detect(cuda_img)

    # For each detection, draw a bounding box and publish the result
    for detection in detections:
        class_desc = net.GetClassDesc(detection.ClassID)
        confidence = detection.Confidence
        bbox = (detection.Left, detection.Top, detection.Right, detection.Bottom)
        cv2.rectangle(cv_img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), 1)
        text = "{}: {:.2f}%".format(class_desc, confidence * 100)
        cv2.putText(cv_img, text, (int(bbox[0]), int(bbox[1])-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # Publish the result
        pub.publish(class_desc)

        rospy.loginfo("Detected '{}' (class #{}) with {:.2f}% confidence".format(class_desc, detection.ClassID, confidence * 100))

def main():
    global net, pub, cv_img

    net = detectNet("ssd-mobilenet-v2")

    rospy.init_node('detectnet_node', anonymous=True)

    sub = rospy.Subscriber('/webcam/image_raw', Image, image_callback)

    pub = rospy.Publisher('/detectnet_result', String, queue_size=10)

    # create a window to display the image
    cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Image", 640, 480)
    
    while not rospy.is_shutdown():
        if cv_img is not None:
            # display the image
            cv2.imshow("Image", cv_img)
        cv2.waitKey(1)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    cv_img = None
    main()
