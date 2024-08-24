#!/usr/bin/env python
# Görev 2: ENU Hareketi
from PGControl import PymavlinkFunctions
import time

if __name__ == "__main__":
    pg = PymavlinkFunctions('udpin:127.0.0.1:14550')

    # GUIDED moda geç ve arm et
    pg.set_mode("GUIDED")
    pg.arm()

    # Kalkış yap
    pg.takeoff(5)
    time.sleep(2)

    # ENU yönünde 3 metre hareket et
    pg.move_enu_with_speed(3, 0, 0, 1)  # Doğu yönüne 3 metre
    pg.move_enu_with_speed(0, 3, 0, 1)  # Kuzey yönüne 3 metre

    # İniş yap ve disarm et
    pg.land()
    pg.disarm()