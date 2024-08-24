#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()

    # Take off to 2 meter height.
    drone.takeoff(5)
    rospy.sleep(3)  # Let the drone stabilize at the desired height

    # Move 2 meters forward while maintaining the current height (2 meters).
    # The drone's reference frame is assumed to have x in the forward direction.
    drone.set_destination(0, 1, 5, 0)

    # Keep the script running until the drone reaches the destination.
    while not drone.check_waypoint_reached():
        rospy.sleep(0.1)

    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
    drone.land()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
