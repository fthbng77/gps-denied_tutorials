#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def publish_grayscale_image():
    rospy.init_node('grayscale_image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/webcam/image_gray/compressed', CompressedImage, queue_size=10)
    bridge = CvBridge()

    def image_callback(msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert the image to grayscale
            gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Convert the grayscale image to a compressed JPEG
            gray_compressed_img = CompressedImage()
            gray_compressed_img.header = msg.header
            gray_compressed_img.format = "jpeg"
            gray_compressed_img.data = np.array(cv2.imencode('.jpg', gray_frame)[1]).tobytes()
            
            # Publish the grayscale compressed image
            image_pub.publish(gray_compressed_img)

            # Display the grayscale image (optional, for debugging purposes)
            cv2.imshow('Grayscale Image', gray_frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    # Subscribe to the webcam topic
    rospy.Subscriber("/webcam/image_raw", Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        publish_grayscale_image()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
