#!/usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import Float32
from pymavlink import mavutil
from pymavlink_functions import PymavlinkFunctions

class MyDrone(PymavlinkFunctions):
    def _init_(self, connection_string):
        super(MyDrone, self)._init_(connection_string)

drone = MyDrone('udpin:localhost:14550')

def distance_callback(msg):
    # distance mesajını işleyin ve drone'un yönünü ayarlayın
    distance_in_pixels = msg.data
    print(distance_in_pixels)
    
    if abs(distance_in_pixels) > 30:
        if distance_in_pixels < 0:
            drone.condition_yaw(1, True, 1)
        elif distance_in_pixels > 0:
            drone.condition_yaw(1, True, -1)


def main():
    rospy.init_node("drone_controller", anonymous=True)
    rospy.Subscriber("/tracking_deviation", Float32, distance_callback)
    drone.arm()
    rospy.sleep(2)
    drone.takeoff(5)
    rospy.sleep(5)    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        drone.send_command(1, 0, 0)  # 1 m/s forward velocity
        print("Moving forward")
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
