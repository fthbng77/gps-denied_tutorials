#!/usr/bin/env python
from pymavlink_functions import PymavlinkFunctions
import time

class MyDrone(PymavlinkFunctions):
    def __init__(self, connection_string):
        super(MyDrone, self).__init__(connection_string)

if __name__ == "__main__":
    # connection_string'in bir UDP portunu dinleyecek şekilde ayarlanması gerekiyor, örneğin: 'udpin:localhost:14551'
    drone = MyDrone('udpin:localhost:14550')

    # GUIDED moda geç ve arm et
    drone.set_mode("GUIDED")
    drone.arm()

    time.sleep(5)

    # 5 metre yüksekliğe kalkış yap
    drone.takeoff(5)
    # Hedef konumda biraz kal
    time.sleep(6)

    # Kare çizdirmeye başla
    current_angle = 0
    for _ in range(4):
        # Yerel koordinatlarda ileri hareket
        drone.send_command(1, 0, 0)
        time.sleep(5)

        # Dronenun yerinde durmasını sağla (hover)
        drone.send_command(0, 0, 0)
        time.sleep(1)
        
        # 90 derece dön
        drone.condition_yaw(90, 1, 1)
        # Dönüşten sonra bir süre bekleyerek hareketten önce dronenun yerinde durmasını sağla
        time.sleep(2)
        
        # Dronenun yerinde durmasını sağla (hover)
        drone.send_command(0, 0, 0)
        time.sleep(1)

    # Drone'u land moduna al
    drone.land()
    time.sleep(5)

    # Drone'u disarm et
    drone.disarm()
