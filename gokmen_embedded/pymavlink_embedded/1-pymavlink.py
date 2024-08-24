#!/usr/bin/env python
# Görev 1: Kalkış ve İniş
from PGControl import PymavlinkFunctions
import time

if __name__ == "__main__":
    # PymavlinkFunctions sınıfını kullanarak drone ile bağlantı kurun
    pg = PymavlinkFunctions('udpin:127.0.0.1:14550')

    # GUIDED moda geç ve arm et
    pg.set_mode("GUIDED")
    pg.arm()
    time.sleep(2)

    # Kalkış yap ve 5 metreye yüksel
    pg.takeoff(5)

    # 5 saniye bekle ve iniş yap
    time.sleep(5)
    pg.land()

    # Drone'u disarm et
    pg.disarm()