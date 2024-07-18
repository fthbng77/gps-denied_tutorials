#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import jetson_inference
import jetson_utils
import numpy as np

class DepthProcessor:

    def __init__(self):
        # ROS Initialization
        rospy.init_node('depth_processing_node', anonymous=True)
        self.bridge = CvBridge()
        self.point_cloud_publisher = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
        self.image_publisher = rospy.Publisher('/processed_image', Image, queue_size=10)
 

        # Jetson Inference Initialization
        self.net = jetson_inference.depthNet()
        rospy.loginfo("DepthNet initialized.")

        self.depth_field = self.net.GetDepthField()
        rospy.loginfo("DepthField retrieved.")

        rospy.sleep(1)
        self.image_subscriber = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_callback)
        rospy.loginfo("Subscribed to /usb_cam/image_raw/compressed.")

        rospy.loginfo("DepthProcessor initialized successfully!")

    def image_callback(self, img_msg):
        if not hasattr(self, 'net'):
            rospy.logerr("Net attribute not initialized!")
            return
            
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        # Convert ROS Image message to CV2 Image
        cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")

        # Convert CV2 Image to CUDA Image
        cuda_image = jetson_utils.cudaFromNumpy(cv_image)

        # Process Image
        self.net.Process(cuda_image)
        jetson_utils.cudaDeviceSynchronize()

        # Update the depth_numpy after processing
        self.depth_numpy = jetson_utils.cudaToNumpy(self.depth_field)
        processed_image = self.depth_numpy_to_displayable_format(self.depth_numpy)

        #cv2.namedWindow('Processed_Image', cv2.WINDOW_NORMAL)

        processed_image_resized = cv2.resize(processed_image, (800, 600)) # Örnek boyut
        cv2.imshow('Processed_Image', processed_image_resized)

        cv2.waitKey(1)
        # 3D coordinates list
        points = []
        for i in range(self.depth_numpy.shape[0]):
            for j in range(self.depth_numpy.shape[1]):
                z = self.depth_numpy[i, j]  # Depth value
                x = i  # x coordinate calculation (modify as needed)
                y = j  # y coordinate calculation (modify as needed)
                points.append([x, y, z])


        cloud_msg = pc2.create_cloud_xyz32(img_msg.header, points)

        # Publish on ROS
        self.point_cloud_publisher.publish(cloud_msg)
        
        
        processed_img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_publisher.publish(processed_img_msg)
        
    def depth_numpy_to_displayable_format(self, depth_numpy):
        normalized_depth = cv2.normalize(depth_numpy, None, 0, 255, cv2.NORM_MINMAX)

        normalized_depth = normalized_depth.astype(np.uint8)

        # Renkli bir görüntüye dönüştürmek için renk haritası uygula
        colored_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

        return colored_depth
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    depth_processor = DepthProcessor()
    depth_processor.run()
