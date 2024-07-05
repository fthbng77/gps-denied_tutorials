#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def publish_grayscale_image():
    rospy.init_node('grayscale_image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/webcam/image_gray', Image, queue_size=10)
    bridge = CvBridge()

    # Initialize variables for optical flow
    prev_gray = None
    transform_accum = np.eye(3, dtype=np.float32)  # Using a 3x3 matrix for affine transformations

    def image_callback(msg):
        nonlocal prev_gray, transform_accum
        try:
            # Convert the ROS Image message to an OpenCV image
            current_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert current frame to grayscale
            current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            
            if prev_gray is not None:
                # Calculate optical flow
                flow = cv2.calcOpticalFlowFarneback(prev_gray, current_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                # Compute the transformation matrix
                dx, dy = flow[..., 0].mean(), flow[..., 1].mean()
                transform_matrix = np.array([[1, 0, dx], [0, 1, dy]], dtype=np.float32)
                
                # Update the accumulated transformation matrix
                transform_accum = transform_accum @ np.vstack([transform_matrix, [0, 0, 1]])
                transform_accum = transform_accum[:2, :]  # Use only the affine part
                
                stabilized_frame = cv2.warpAffine(current_frame, transform_accum, (current_frame.shape[1], current_frame.shape[0]))

                # Convert the stabilized frame to grayscale
                gray_frame = cv2.cvtColor(stabilized_frame, cv2.COLOR_BGR2GRAY)
            else:
                gray_frame = current_gray
            
            # Update previous frame
            prev_gray = current_gray

            # Convert the grayscale image to a ROS Image message
            gray_image_msg = bridge.cv2_to_imgmsg(gray_frame, encoding="mono8")
            gray_image_msg.header = msg.header
            
            # Publish the grayscale image
            image_pub.publish(gray_image_msg)

            # Display the grayscale image (optional, for debugging purposes)
            cv2.imshow('Stabilized Grayscale Image', gray_frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    # Subscribe to the webcam topic
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        publish_grayscale_image()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
