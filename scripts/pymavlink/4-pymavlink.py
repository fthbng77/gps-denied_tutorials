#!/usr/bin/env python
from PGControl import PymavlinkFunctions
import time

if __name__ == "__main__":
    pg = PymavlinkFunctions('udpin:127.0.0.1:14550')

    pg.set_mode("GUIDED")
    pg.arm()

    pg.takeoff(5)
    time.sleep(2)

    # Waypoint listesini tanımla (enlem, boylam, irtifa)
    waypoints = [
        (-35.363423, 149.165157, 10),  # Birinci waypoint: 10 metre irtifa
        (-35.363236, 149.164753, 15)   # İkinci waypoint: 15 metre irtifa
    ]

    # Waypoint'lere git ve görevi başlat
    pg.goto_gps(waypoints)

    # Waypoint uçuşu tamamlanana kadar bekleyin
    time.sleep(60)

    # İniş yap
    pg.land()

    # İnişi tamamlamak için bekle
    time.sleep(10)
