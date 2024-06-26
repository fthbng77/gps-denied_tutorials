#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam/image_gray/compressed", CompressedImage, self.callback)
        self.distance_pub = rospy.Publisher("/tracking_deviation", Float32, queue_size=10)  # Create a publisher

    def callback(self, data):
        # Convert the JPEG-compressed image from ROS message to cv2 object
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        # Get the portion of the image to be segmented
        x1, y1, x2, y2 = 0, 380, 639, 479
        roi = cv_image[y1:y2, x1:x2]

        # Define the range of colors for the path (grayscale range)
        lower_gray = 80
        upper_gray = 255

        # Mask the colors in the specified range
        mask = cv2.inRange(roi, lower_gray, upper_gray)

        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        M = cv2.moments(mask)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)  # Convert back to BGR for drawing
            cv2.circle(cv_image, (cx, cy), 5, (255, 0, 0), -1)

            # Calculate and publish the deviation
            center_x_crosshair = roi.shape[1] // 2
            distance = center_x_crosshair - cx
            print("Tracking Deviation: ", distance)
            self.distance_pub.publish(distance)

        # Apply the mask to the original image
        masked_img = cv2.bitwise_and(roi, roi, mask=mask)

        # Detect edges
        edges = cv2.Canny(mask, 50, 150)

        # Find contours
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours
        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            for i in range(len(approx) - 1):
                cv2.line(cv_image, tuple(approx[i][0]), tuple(approx[i + 1][0]), (255, 255, 0), 3)
            cv2.line(cv_image, tuple(approx[0][0]), tuple(approx[len(approx) - 1][0]), (255, 255, 0), 3)

        # Draw crosshair
        center_x = cv_image.shape[1] // 2
        center_y = cv_image.shape[0] // 2
        color = (0, 0, 255)
        thickness = 2
        x_length = 10
        y_length = 10
        cv2.line(cv_image, (center_x, center_y - y_length), (center_x, center_y + y_length), color, thickness)
        cv2.line(cv_image, (center_x - x_length, center_y), (center_x + x_length, center_y), color, thickness)

        # Display the original full image
        cv2.rectangle(cv_image, (0, 380), (639, 479), (200, 200, 200), 2)
        cv2.imshow("cv2 Segmentation", cv_image)

        # Display the masked image segment
        cv2.imshow("cv2_Segmented ROI Image", masked_img)

        cv2.waitKey(1)
        
if __name__ == "__main__":
    rospy.init_node('image_converter', anonymous=True)
    ic = ImageConverter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()