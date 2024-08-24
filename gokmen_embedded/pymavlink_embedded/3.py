#! /usr/bin/env python
import rospy
from iq_gnc.py_gnc_functions import *
from iq_gnc.PrintColours import *

def main():
    rospy.init_node("drone_controller", anonymous=True)
    drone = gnc_api()
    drone.set_mode("GUIDED")
    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3) 
    rate = rospy.Rate(3)

    goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
             [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
