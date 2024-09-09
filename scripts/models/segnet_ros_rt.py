#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from jetson_inference import segNet
from jetson_utils import cudaOverlay, cudaDeviceSynchronize, Log
from segnet_utils import *
from cv_bridge import CvBridge
from jetson_utils import cudaFromNumpy, cudaToNumpy
import cv2
class Args:
    def __init__(self):
        self.stats = False
        self.visualize = "overlay"
        self.alpha = 150
        self.mask = None

args = Args()

network = "fcn-resnet18-cityscapes-512x256"
alpha = 150 # 0 to 255

def image_callback(data):
    global img_format
    img_input = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    if img_format is None:
        img_format = data.encoding
        buffers.Alloc(img_input.shape, img_format)
    else:
        img_input = cudaFromNumpy(img_input)
    net.Process(img_input)
    if buffers.overlay:
        net.Overlay(buffers.overlay)
    if buffers.mask:
        net.Mask(buffers.mask)
    if buffers.composite:
        cudaOverlay(buffers.overlay, buffers.composite, 0, 0)
        cudaOverlay(buffers.mask, buffers.composite, buffers.overlay.width, 0)
    cudaDeviceSynchronize()
    net.PrintProfilerTimes()
    buffers.ComputeStats()
    img_output = cudaToNumpy(buffers.output)
    img_output_rgb = cv2.cvtColor(img_output, cv2.COLOR_RGBA2RGB)
    pub.publish(bridge.cv2_to_imgmsg(img_output_rgb, "rgb8"))

def main():
    global net, pub, bridge, img_format, buffers

    rospy.init_node('segnet_node')
    net = segNet(network)
    net.SetOverlayAlpha(alpha)

    buffers = segmentationBuffers(net, args)
    img_format = None
    bridge = CvBridge()

    pub = rospy.Publisher('/segnet_image', Image, queue_size=10)
    rospy.Subscriber("/webcam/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

