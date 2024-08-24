#!/usr/bin/env python
from gokmen.scripts.pymavlink.PGControl import PymavlinkFunctions
import time

# Main kısmı
if __name__ == "__main__":
    # connection_string'in bir UDP portunu dinleyecek şekilde ayarlanması gerekiyor, örneğin: 'udpin:localhost:14550'
    pg = PymavlinkFunctions('udpin:127.0.0.1:14550')

    # GUIDED moda geç ve arm et
    pg.set_mode("GUIDED")
    pg.arm()

    # Kalkış yap ve 5 metreye yüksel
    pg.takeoff(5)

    # Kalkıştan sonra drone'u stabilize etmek için bekleme süresi
    time.sleep(2)

    # 1. bölgede 3m uzunluğunda bir kare çiz
    side_length = 3  # Karenin kenar uzunluğu (metre)
    speed = 1  # Hız (m/s)

    # İlk kenar için hareket (Kuzey yönünde)
    pg.move_enu_with_speed(0, side_length, 0, speed)
    
    # Yaw ile 90 derece döndür (Saat yönü)
    pg.turn(90, speed=30, clockwise=True)
    time.sleep(1)  # Dönüşü tamamlamak için kısa bekleme

    # İkinci kenar için hareket (Doğu yönünde)
    pg.move_enu_with_speed(side_length, 0, 0, speed)
    
    # Yaw ile 90 derece döndür (Saat yönü)
    pg.turn(90, speed=30, clockwise=True)
    time.sleep(1)  # Dönüşü tamamlamak için kısa bekleme

    # Üçüncü kenar için hareket (Güney yönünde)
    pg.move_enu_with_speed(0, -side_length, 0, speed)
    
    # Yaw ile 90 derece döndür (Saat yönü)
    pg.turn(90, speed=30, clockwise=True)
    time.sleep(1)  # Dönüşü tamamlamak için kısa bekleme

    # Dördüncü kenar için hareket (Batı yönünde)
    pg.move_enu_with_speed(-side_length, 0, 0, speed)
    
    # Yaw ile 90 derece döndür (Saat yönü)
    pg.turn(90, speed=30, clockwise=True)
    time.sleep(1)  # Dönüşü tamamlamak için kısa bekleme

    # Drone'u land moduna al
    pg.land()

    # Drone'u disarm et
    pg.disarm()