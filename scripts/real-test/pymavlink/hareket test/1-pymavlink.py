#!/usr/bin/env python
from pymavlink_functions import PymavlinkFunctions
import time

class MyDrone(PymavlinkFunctions):
    def __init__(self, connection_string):
        super(MyDrone, self).__init__(connection_string)

if __name__ == "__main__":
    drone = MyDrone('udpin:localhost:14550')

    drone.set_mode("GUIDED")
    drone.arm()

    drone.takeoff(2)
    time.sleep(5)

    # Drone'u land moduna al
    drone.land()

    # Drone'u disarm et
    drone.disarm()
