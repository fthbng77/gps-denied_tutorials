#!/usr/bin/env python

import rospy
import cv2
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Download the MiDaS
device = "cuda" if torch.cuda.is_available() else "cpu"
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
midas.to(device)
midas.eval()

# Input transformation pipeline
transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = transforms.small_transform

# Convert ROS Image messages to OpenCV format
bridge = CvBridge()

def process_image(frame):
    # Transform input for midas
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    imgbatch = transform(img).to(device)

    # Make prediction
    with torch.no_grad():
        prediction = midas(imgbatch)
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False
        ).squeeze()

        return prediction.cpu().numpy()

def display_image(frame, depth_output):
    # Normalize the depth map to be in the range 0-255
    depth_normalized = cv2.normalize(depth_output, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
    # Apply the chosen colormap
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_BONE)
    
    display_image = cv2.hconcat([frame, depth_colored])
    cv2.imshow("MiDaS Depth Estimation", display_image)

def image_callback(img_msg):
    # Check if the image message is valid
    if img_msg is None:
        rospy.logwarn("Received an empty image!")
        return

    # Convert image from ROS to OpenCV format
    frame = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    
    # Process the image and get the depth prediction
    depth_output = process_image(frame)
    
    # Display the original image and depth prediction
    display_image(frame, depth_output)
    
    if cv2.waitKey(10) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User terminated the process.")

def main():
    rospy.init_node('midas_depth_node')

    # Subscribe to the webcam topic
    rospy.Subscriber('/webcam/image_raw', Image, image_callback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()