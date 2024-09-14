#!/usr/bin/env python
# Görev 2: NED Hareketi (Kare Çizme)
from PGControl import PymavlinkFunctions
import time

if __name__ == "__main__":
    pg = PymavlinkFunctions('udpin:127.0.0.1:14550')

    pg.set_mode("GUIDED")
    pg.arm()

    pg.takeoff(5)
    time.sleep(2)

    side_length = 3  # Kare kenar uzunluğu (metre)
    speed = 1  # Hız (m/s)
    duration = side_length / speed  # Hareket süresi = mesafe / hız

    # 1. Adım: Kuzey yönünde (ileri)
    pg.move_NED(north=side_length, east=0, down=0, duration=duration)
    time.sleep(duration)  # Hareket süresini bekle

    # 90 derece sağa dön (yaw komutu ile)
    pg.turn(90, speed=30, clockwise=True, relative=True)
    time.sleep(2)

    # 2. Adım: Doğu yönünde (ileri, ama drone yön değiştirdi)
    pg.move_NED(north=side_length, east=0, down=0, duration=duration)
    time.sleep(duration)

    # Yine 90 derece sağa dön
    pg.turn(90, speed=30, clockwise=True, relative=True)
    time.sleep(2)

    # 3. Adım: Güney yönünde (ileri, ama drone yön değiştirdi)
    pg.move_NED(north=side_length, east=0, down=0, duration=duration)
    time.sleep(duration)

    # Yine 90 derece sağa dön
    pg.turn(90, speed=30, clockwise=True, relative=True)
    time.sleep(2)

    # 4. Adım: Batı yönünde (ileri, ama drone yön değiştirdi)
    pg.move_NED(north=side_length, east=0, down=0, duration=duration)
    time.sleep(duration)

    # Kare tamamlandı, inişe geç
    pg.land()

    # İnişi tamamlamak için bekle
    time.sleep(10)
