#!/usr/bin/env python
import sys
import os

# 'scripts' klasörünün yolunu al (örneğin: /home/yusuf/catkin_ws/src/iq_gnc/scripts)
scripts_folder = os.path.dirname(os.path.realpath(__file__))

# 'pymavlink' klasörünün yolunu 'scripts' klasörüne ekleyerek oluştur
pymavlink_folder = os.path.join(scripts_folder, 'pymavlink')

# 'pymavlink' klasörünün yolunu sys.path'e ekleyin
sys.path.append(pymavlink_folder)

# Şimdi pymavlink_functions modülünü içe aktarabilirsiniz
from pymavlink_functions import PymavlinkFunctions
import rospy
import time
from iq_gnc.py_gnc_functions import *
from iq_gnc.PrintColours import *

class MyController:
    def __init__(self):
        # MAVROS ile drone'a bağlan
        self.gnc_api_drone = gnc_api()
        self.gnc_api_drone.wait4connect()
        self.gnc_api_drone.wait4start()
        #self.gnc_api_drone.initialize_local_frame()

        # pymavlink ile drone'a bağlan
        self.pymavlink_drone = PymavlinkFunctions('udpin:localhost:14551') # Portu drone'unuz için doğru olanla değiştirin

    def start(self):
        self.pymavlink_drone.arm()

        # Kalkış yap
        self.pymavlink_drone.takeoff(2)
        # Hedef konumda biraz kal
        time.sleep(5)

        self.pymavlink_drone.move(5,0,0)
        time.sleep(5)

        # Drone'u land moduna al
        self.pymavlink_drone.land()

        # Drone'u disarm et
        self.pymavlink_drone.disarm()

if __name__ == "__main__":
    rospy.init_node("drone_controller", anonymous=True)
    controller = MyController()
    controller.start()
