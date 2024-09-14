#!/usr/bin/env python
from PGControl import PymavlinkFunctions
import time

if __name__ == "__main__":
    pg = PymavlinkFunctions('udpin:127.0.0.1:14550')

    pg.set_mode("GUIDED")
    pg.arm()

    pg.takeoff(5)  # 5 metre irtifaya kalkış
    time.sleep(2)

    # Kare hareketini başlat
    side_length = 3  # Kare kenar uzunluğu (metre)

    # 1. Adım: Kuzey yönünde (ileri) - Hız 1 m/s
    pg.set_speed(1, b"WPNAV_SPEED")  # Hız ayarla: 1 m/s
    duration = side_length / 1  # Süre = mesafe / hız
    pg.move_NED(north=side_length, east=0, down=0, duration=duration)
    time.sleep(duration)  # Hareket süresini bekle

    # 90 derece sağa dön (yaw komutu ile)
    pg.turn(90, speed=30, clockwise=True, relative=True)
    time.sleep(2)

    # 2. Adım: Doğu yönünde - Hız 2 m/s
    pg.set_speed(2, b"WPNAV_SPEED")  # Hız ayarla: 2 m/s
    duration = side_length / 2  # Süre = mesafe / hız
    pg.move_NED(north=side_length, east=0, down=0, duration=duration)
    time.sleep(duration)

    # Yine 90 derece sağa dön
    pg.turn(90, speed=30, clockwise=True, relative=True)
    time.sleep(2)

    # 3. Adım: Güney yönünde - Hız 0.5 m/s (yavaş hareket)
    pg.set_speed(0.5, b"WPNAV_SPEED")  # Hız ayarla: 0.5 m/s
    duration = side_length / 0.5  # Süre = mesafe / hız
    pg.move_NED(north=side_length, east=0, down=0, duration=duration)
    time.sleep(duration)

    # Yine 90 derece sağa dön
    pg.turn(90, speed=30, clockwise=True, relative=True)
    time.sleep(2)

    # 4. Adım: Batı yönünde - Hız 3 m/s (hızlı hareket)
    pg.set_speed(3, b"WPNAV_SPEED")  # Hız ayarla: 3 m/s
    duration = side_length / 3  # Süre = mesafe / hız
    pg.move_NED(north=side_length, east=0, down=0, duration=duration)
    time.sleep(duration)

    # Kare tamamlandı, inişe geç
    pg.land()

    # İnişi tamamlamak için bekle
    time.sleep(10)
