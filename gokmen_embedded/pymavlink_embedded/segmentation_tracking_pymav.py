#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Float32
from pymavlink import mavutil
from pymavlink_functions import PymavlinkFunctions


class MyDrone(PymavlinkFunctions):
    def __init__(self, connection_string):
        super(MyDrone, self).__init__(connection_string)

# drone değişkenini global olarak tanımlayın
# Cube Orange ile bağlantıyı başlat
drone = MyDrone('udpin:localhost:14551')


def distance_callback(msg):
    # distance mesajını işleyin ve drone'un yönünü ayarlayın
    distance_in_pixels = msg.data
    print(distance_in_pixels)
    
    # Eğer distance değeri 0'dan küçükse, drone'un saat yönünün tersine dönmesi gerekmektedir (negatif yaw değeri)
    if (abs(distance_in_pixels)>10):
        if (distance_in_pixels < 0):
            drone.condition_yaw(1,True,1)
        elif (distance_in_pixels > 0):
            drone.condition_yaw(1,True,-1)

def roll_adjustment_callback(msg):
    roll_adjustment = msg.data
    print("Roll adjustment: ", roll_adjustment)
    # If the drone is too close to the left edge of the road, move it to the right
    if roll_adjustment == -1:
        drone.move(0, 1, 0)  # Move right
    # If the drone is too close to the right edge of the road, move it to the left
    elif roll_adjustment == 1:
        drone.move(0, -1, 0)  # Move left

def main():
    rospy.init_node("drone_controller", anonymous=True)
    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(5)
    rospy.sleep(7)

    # /distance_in_pixels konusuna abone olun
    rospy.Subscriber("/distance_in_pixels", Float32, distance_callback)
    rospy.Subscriber("/roll_adjustment", Float32, roll_adjustment_callback)



    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass