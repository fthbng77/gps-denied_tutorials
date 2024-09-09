#!/usr/bin/env python
from pymavlink_functions import PymavlinkFunctions
import time

class MyDrone(PymavlinkFunctions):
    def __init__(self, connection_string):
        super(MyDrone, self).__init__(connection_string)

if __name__ == "__main__":
    # connection_string'in bir UDP portunu dinleyecek şekilde ayarlanması gerekiyor, örneğin: 'udpin:localhost:14551'
    drone = MyDrone('udpin:localhost:14551')

    # GUIDED moda geç ve arm et
    drone.set_mode("GUIDED")
    drone.arm()

    # Kalkış yap
    drone.takeoff(2)
    # Hedef konumda biraz kal
    time.sleep(5)

    drone.move_relative(2,0,0)
    time.sleep(5)

    # Drone'u land moduna al
    drone.land()

    # Drone'u disarm et
    drone.disarm()
