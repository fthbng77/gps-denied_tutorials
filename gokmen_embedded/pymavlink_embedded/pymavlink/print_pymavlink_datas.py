#!/usr/bin/env python
from pymavlink_functions import PymavlinkFunctions
import time

class MyDrone(PymavlinkFunctions):
    def __init__(self, connection_string):
        super(MyDrone, self).__init__(connection_string)

if __name__ == "__main__":
    # Seri port ve baudrate'i tek bir bağlantı dizesinde birleştirin
    connection_string = 'tcp:localhost:14550'

    # MyDrone sınıfından bir örnek oluşturun ve bağlantı dizesini aktarın
    drone = MyDrone(connection_string)

    while True:
        # Yeni bir mesajı bekleyin ve yazdırın
        msg = drone.drone.recv_match(blocking=True)
        print(msg)
