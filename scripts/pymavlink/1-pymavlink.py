#!/usr/bin/env python
# Görev 1: Kalkış ve İniş
from PGControl import PymavlinkFunctions
import time

if __name__ == "__main__":
    pg = PymavlinkFunctions('udpin:127.0.0.1:14550')

    pg.set_mode("GUIDED")
    pg.arm()
    time.sleep(2)

    pg.takeoff(5)

    time.sleep(5)
    pg.land()

    pg.disarm()
