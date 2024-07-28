#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.distance_pub = rospy.Publisher("/tracking_deviation", Float32, queue_size=10)  # Create a publisher

    def callback(self, data):
        # Convert the image from ROS message to cv2 object
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Get the portion of the image to be segmented
        x1, y1, x2, y2 = 0, 380, 639, 479
        roi = cv_image[y1:y2, x1:x2]
        # Görüntüyü HSV renk uzayına dönüştür
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # yol için sınır değerleri belirle
        lower_hsv = (0, 0, 50)
        upper_hsv = (179, 50, 255)

        # Belirlenen sınırlar arasındaki renkler maskelenir
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Morfolojik işlem uygula
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        M = cv2.moments(mask)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        cv2.circle(roi, (cx,cy),5,(255,0,0),-1)

        # Maske görüntüsü orijinal görüntü ile çarpılır
        masked_img = cv2.bitwise_and(roi, roi, mask=mask)

        # Gri tonlamalı resim
        gray = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)

        # Kenarlar algılanır
        edges = cv2.Canny(gray, 50, 150)

        # Görüntüdeki konturlar tespit edilir
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Konturlar çizdirilir
        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            for i in range(len(approx) - 1):
                cv2.line(roi, tuple(approx[i][0]), tuple(approx[i + 1][0]), (255, 255, 0), 3)
            cv2.line(roi, tuple(approx[0][0]), tuple(approx[len(approx) - 1][0]), (255, 255, 0), 3)

        # Crosshair çiz
        center_x = roi.shape[1] // 2  # Use the width of the ROI
        center_y = roi.shape[0] // 2  # Use the height of the ROI
        color = (0, 0, 255)  # Red color
        thickness = 2
        x_length = 10
        y_length = 10
        # Vertical line
        cv2.line(roi, (center_x, center_y - y_length), (center_x, center_y + y_length), color, thickness)
        # Horizontal line
        cv2.line(roi, (center_x - x_length, center_y), (center_x + x_length, center_y), color, thickness)

        # Moment'in merkezini hesapla
        M = cv2.moments(mask)
        if M["m00"] != 0:
            cx_moment = int(M["m10"]/M["m00"])
            cy_moment = int(M["m01"]/M["m00"])
            cv2.circle(roi, (cx_moment,cy_moment),5,(255,0,0),-1)

            # Crosshair'ın merkezini hesapla
            center_x_crosshair = roi.shape[1] // 2
            center_y_crosshair = roi.shape[0] // 2

            # Crosshair'ın merkezi ile momentin merkezi arasında bir çizgi çiz
            cv2.line(roi, (center_x_crosshair, center_y_crosshair), (cx_moment, cy_moment), (110, 0, 100), 2)

            distance = center_x_crosshair - cx_moment
            print("Tracking Deviation: ", distance)
            
            # Publish the distance
            self.distance_pub.publish(distance)

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
