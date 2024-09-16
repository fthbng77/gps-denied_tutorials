
from pymavlink import mavutil
from PrintColours import PrintColours
import time
import math

class PymavlinkFunctions:
    def __init__(self, connection_string=None):
        # Connect to the pg only if connection_string is provided
        if connection_string is not None:
            self.pg = mavutil.mavlink_connection(connection_string)
            # Wait for the pg to connect
            msg = self.pg.wait_heartbeat()
            PrintColours.print_info("Heartbeat from system (system %u component %u)" %
                                    (self.pg.target_system, self.pg.target_component))
        else:
            self.pg = None  # You can set it to None or some default value

    def set_mode(self, mode):
        # pg'un uçuş modunu ayarla
        PrintColours.print_info(f"{mode} moduna geçiliyor")

        try:
            # Mode haritasını al
            mode_mapping = self.pg.mode_mapping()
            mode_id = mode_mapping.get(mode)

            # Eğer geçerli bir mod varsa komutu gönder
            if mode_id is not None:
                self.pg.mav.set_mode_send(
                    self.pg.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
                
                # Modun değiştirildiğini doğrulamak için bir süre bekle
                ack_msg = self.pg.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
                if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    PrintColours.print_info(f"{mode} moduna başarıyla geçildi")
                    
                    # HEARTBEAT mesajını kontrol et, mod değişikliğini doğrula
                    while True:
                        heartbeat_msg = self.pg.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
                        custom_mode = heartbeat_msg.custom_mode
                        if custom_mode == mode_id:
                            PrintColours.print_info(f"{mode} moduna başarıyla geçildi")
                            break
                        time.sleep(1)
                else:
                    PrintColours.print_error(f"{mode} moduna geçiş başarısız")

            else:
                PrintColours.print_warning(f"Bilinmeyen mod: {mode}")

        except Exception as e:
            PrintColours.print_error(f"Mod değiştirirken hata oluştu: {e}")

    def arm(self):
        # Arm the pg
        PrintColours.print_info("Arming the pg")
        
        try:
            # Arm komutunu gönder
            self.pg.mav.command_long_send(
                self.pg.target_system, self.pg.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
            )
            
            # Komutun ACK mesajını bekle
            ack_msg = self.pg.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            
            # Komutun kabul edilip edilmediğini kontrol et
            if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                PrintColours.print_info("pg armed successfully")
                
                # HEARTBEAT mesajında arm durumunu kontrol et
                while True:
                    heartbeat_msg = self.pg.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
                    if heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        PrintColours.print_info("pg is confirmed armed")
                        break
            else:
                PrintColours.print_error("Failed to arm the pg")

        except Exception as e:
            PrintColours.print_error(f"Error during arming: {e}")

    def disarm(self):
        # pg'u disarm et
        PrintColours.print_blinking_warning("pg'un disarm edilmesi bekleniyor")
        time.sleep(3)
        
        # Disarm komutu gönder
        PrintColours.print_info("pg disarm ediliyor")
        self.pg.mav.command_long_send(
            self.pg.target_system, self.pg.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )

        # Komutun kabul edildiğini kontrol et
        ack_msg = self.pg.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            PrintColours.print_info("pg başarıyla disarm edildi")

            # Disarm işleminin doğruluğunu kontrol etmek için HEARTBEAT mesajını izleyin
            while True:
                heartbeat_msg = self.pg.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
                if not (heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    PrintColours.print_info("pg disarm modunda")
                    break
                time.sleep(1)
        else:
            PrintColours.print_error("pg disarm edilemedi")

    def takeoff(self, altitude=5):
        # pg'un belirtilen irtifaya kalkmasını sağla (metre cinsinden)
        PrintColours.print_blinking_warning("pg'un kalkış yapması bekleniyor")
        time.sleep(2)
        PrintColours.print_info("Kalkış yapılıyor")

        try:
            # Kalkış komutunu gönder
            self.pg.mav.command_long_send(
                self.pg.target_system, self.pg.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
            )

            # Komutun ACK mesajını bekle
            ack_msg = self.pg.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            
            # Komutun kabul edilip edilmediğini kontrol et
            if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                PrintColours.print_info("pg başarıyla kalkış yaptı")

                # HEARTBEAT mesajını kontrol ederek irtifa kontrolü
                while True:
                    position_msg = self.pg.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
                    current_altitude = position_msg.relative_alt / 1000.0  # Altitude değerini metreye çevir
                    if current_altitude >= altitude * 0.95:  # Hedef irtifanın %95'ine ulaşıldığında durdur
                        PrintColours.print_info(f"pg hedef irtifaya ({altitude} metre) başarıyla ulaştı")
                        break
                    time.sleep(1)
            else:
                PrintColours.print_error("pg kalkış yapamadı")

        except Exception as e:
            PrintColours.print_error(f"Kalkış sırasında hata oluştu: {e}")

    def land(self):
        # pg'u indir
        PrintColours.print_info("pg inişe geçiyor")
        
        # LAND modu için MAVLink kodu 9'dur
        LAND_MODE = 9
        
        try:
            # LAND modunu ayarlamak için komut gönder
            self.pg.mav.set_mode_send(
                self.pg.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                LAND_MODE
            )

            # Komutun kabul edildiğini kontrol et
            ack_msg = self.pg.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                PrintColours.print_info("pg iniş moduna başarıyla geçti")

                # LAND moduna geçildiğini doğrulamak için HEARTBEAT mesajını izleyin
                while True:
                    heartbeat_msg = self.pg.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
                    if heartbeat_msg.custom_mode == LAND_MODE:
                        PrintColours.print_info("pg iniş modunda")
                        break
                    time.sleep(1)
            else:
                PrintColours.print_error("pg iniş moduna geçemedi")

        except Exception as e:
            PrintColours.print_error(f"İniş sırasında hata oluştu: {e}")


    def move_NED(self, north, east, down, duration):
        """
        PG'yi NED (North-East-Down) yönünde belirli bir süre boyunca hızlandırır.
        
        Parameters:
        north (float): Kuzey yönünde hız (m/s).
        east (float): Doğu yönünde hız (m/s).
        down (float): Aşağı yönünde hız (m/s).
        duration (float): Hareket süresi (saniye cinsinden).
        """
        # NED çerçevesinde hızları ayarlama (Body Frame: Gövdeye göre referans alıyor)
        self.pg.mav.set_position_target_local_ned_send(
            0,  # Zaman damgası (genellikle 0 yapılır)
            self.pg.target_system,  # Hedef sistem
            self.pg.target_component,  # Hedef bileşen
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # NED çerçevesi, drone gövdesine göre
            0b0000111111000111,  # Tip maskesi (sadece hızları kontrol ediyoruz)
            0, 0, 0,  # Pozisyon kontrol edilmiyor
            north, east, down,  # Hız değerleri (NED: Kuzey, Doğu, Aşağı)
            0, 0, 0,  # Açısal hızlar kontrol edilmiyor
            0, 0  # Yaw ve yaw hızları sıfır
        )

        # Belirtilen süre boyunca hareket et
        time.sleep(duration)

        # Hareketi durdurmak için hızları sıfırla
        self.pg.mav.set_position_target_local_ned_send(
            0,
            self.pg.target_system,
            self.pg.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,  # Pozisyon sıfırlanıyor
            0, 0, 0,  # Hızlar sıfır
            0, 0, 0,  # Açısal hızlar sıfır
            0, 0  # Yaw ve yaw hızları sıfır
        )

    def goto_gps(self, waypoints, wait_for_ack=True):
        """
        Birden fazla GPS koordinatına (enlem, boylam, irtifa) hareket ettirir ve waypoint görevini başlatır.

        Parameters:
        waypoints (list): [(lat, lon, alt), ...] şeklinde bir waypoint listesi.
        wait_for_ack (bool): Komutun kabul edilip edilmediğini bekler. Varsayılan True.
        """
        PrintColours.print_info(f"Sending waypoints: {len(waypoints)}")

        # Waypoint listesi hazırlanıyor
        wp_list = []
        seq = 0

        # Görevleri ekleyin
        for waypoint in waypoints:
            lat, lon, alt = waypoint
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp_list.append(mavutil.mavlink.MAVLink_mission_item_message(
                self.pg.target_system,
                self.pg.target_component,
                seq,  # Görev sırası
                frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Waypoint komutu
                0, 0, 0, 0, 0, 0,
                lat,  # Enlem
                lon,  # Boylam
                alt   # İrtifa
            ))
            seq += 1  # Sonraki waypoint için sıralamayı arttır

        # Eski waypoint'leri temizleyin
        self.pg.mav.mission_clear_all_send(self.pg.target_system, self.pg.target_component)
        PrintColours.print_info("Previous mission cleared")

        # Görev sayısını gönderin
        self.pg.mav.mission_count_send(self.pg.target_system, self.pg.target_component, len(wp_list))
        PrintColours.print_info(f"Mission count: {len(wp_list)} sent")

        # Her bir waypoint'i gönderin
        for i in range(len(wp_list)):
            msg = self.pg.recv_match(type="MISSION_REQUEST", blocking=True, timeout=10)
            if msg is None:
                PrintColours.print_error("Timeout while waiting for MISSION_REQUEST")
                return
            if msg.seq == i:
                self.pg.mav.send(wp_list[i])
                PrintColours.print_info(f"Waypoint {i} sent")

        if wait_for_ack:
            # Görev onayının beklenmesi
            ack_msg = self.pg.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if ack_msg and ack_msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                PrintColours.print_info("Mission acknowledged by the pg")
            else:
                PrintColours.print_error("Failed to receive mission acknowledgment")
                return

        # AUTO moda geçerek waypoint görevini başlat
        self.set_mode("AUTO")
        time.sleep(2)

        # Görev başlatma komutu gönder
        self.pg.mav.command_long_send(
            self.pg.target_system,
            self.pg.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,  # Waypoint görevini başlat
            0,  # confirmation
            0,  # first_item (0 = ilk waypoint)
            0,  # last_item (0 = son waypoint)
            0, 0, 0, 0, 0  # diğer parametreler kullanılmıyor
        )
        PrintColours.print_info("Mission started")

    def turn(self, angle, speed=30, clockwise=True, relative=True):
        """
        Dronu belirli bir açıya döndürür (yaw komutu).

        Parameters:
        angle (float): Dönmek istenen açı (derece cinsinden).
        speed (float): Dönüş hızı (derece/saniye cinsinden). Default 30.
        clockwise (bool): Dönüş yönü. True: Saat yönünde, False: Saat yönünün tersinde. Default True.
        relative (bool): Dönüş açısının mutlak mı yoksa göreceli mi olduğu. True: Göreceli, False: Mutlak. Default True.
        """
        PrintColours.print_info(f"Turning pg by {angle} degrees at {speed} degrees/sec")

        # Dönüş yönü belirleniyor. Saat yönünde ise 1, saat yönünün tersinde ise -1.
        direction = 1 if clockwise else -1

        # Göreceli mi mutlak mı kontrolü
        is_relative = 1 if relative else 0

        # MAV_CMD_CONDITION_YAW komutunu gönder
        self.pg.mav.command_long_send(
            self.pg.target_system, self.pg.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            angle,  # Hedef açı
            speed,  # Dönüş hızı
            direction,  # Dönüş yönü (1: saat yönü, -1: saat yönünün tersi)
            is_relative,  # 0: Mutlak açı, 1: Göreceli açı
            0, 0, 0  # Kullanılmayan diğer parametreler
        )

        # Komutun etkili olabilmesi için bekleme
        time.sleep(1)
   
    def get_attitude(self):
        """
        pg'un anlık attitude (durum) verilerini alır.

        Returns:
        tuple: Roll, pitch ve yaw değerlerini (radyan cinsinden) döndürür.
        """
        PrintColours.print_info("Attitude bilgisi alınıyor...")

        # ATTITUDE mesajını bekle
        attitude_msg = self.pg.recv_match(type='ATTITUDE', blocking=True, timeout=5)

        # Mesajın alınmadığı durumu kontrol et
        if attitude_msg is None:
            PrintColours.print_error("ATTITUDE mesajı alınamadı!")
            return None, None, None

        # roll, pitch ve yaw değerlerini döndür
        return attitude_msg.roll, attitude_msg.pitch, attitude_msg.yaw
        
    def get_position(self):
        """
        pg'un mevcut global pozisyonunu alır.

        Returns:
        tuple: pg'un enlem, boylam ve irtifasını içeren bir tuple (latitude, longitude, altitude).
        """
        PrintColours.print_info("pg'un pozisyon bilgisi alınıyor...")
        
        # GLOBAL_POSITION_INT mesajını bekle
        msg = self.pg.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)

        # Mesajın geldiğini kontrol et
        if msg is None:
            PrintColours.print_error("pg'dan pozisyon bilgisi alınamadı!")
            return None

        # Latitude, longitude ve altitude değerlerini mesajdan çıkart
        latitude = msg.lat / 1e7  # derece cinsinden enlem
        longitude = msg.lon / 1e7  # derece cinsinden boylam
        altitude = msg.alt / 1e3  # metre cinsinden irtifa

        # Enlem, boylam ve irtifa bilgilerini döndür
        return latitude, longitude, altitude

    def check_waypoint_reached(self, waypoint, tolerance=1):
        """
        pg'un waypoint'e belirli bir tolerans aralığında ulaşıp ulaşmadığını kontrol eder.

        Parameters:
        waypoint (list or tuple): Hedef waypoint [latitude, longitude, altitude] şeklinde.
        tolerance (float): pg'un waypoint'e ulaşmış sayılması için tolerans mesafesi. Varsayılan 1 metredir.

        Returns:
        bool: pg waypoint'e ulaştıysa True, ulaşmadıysa False döner.
        """
        current_position = self.get_position()  # [latitude, longitude, altitude] döner
        
        # Pozisyonun alınıp alınmadığını kontrol et
        if current_position is None:
            PrintColours.print_error("pg'un mevcut pozisyonu alınamadı!")
            return False

        # Mesafe hesabı (Haversine yerine basit mesafe formülü kullanılıyor)
        distance = ((current_position[0] - waypoint[0])**2 + 
                    (current_position[1] - waypoint[1])**2 + 
                    (current_position[2] - waypoint[2])**2)**0.5

        # Waypoint'e ulaşılıp ulaşılmadığını kontrol et
        if distance <= tolerance:
            PrintColours.print_info("Waypoint'e ulaşıldı.")
            return True
        else:
            PrintColours.print_warning(f"Waypoint'e ulaşılamadı. Şu anki mesafe: {distance:.2f} metre")
            return False

    def get_alt(self):
        """
        PG'nin mevcut irtifasını (relative altitude) alır.
        
        Returns:
        float: İrtifa (metre cinsinden)
        """
        PrintColours.print_info("İrtifa bilgisi alınıyor...")

        # GLOBAL_POSITION_INT mesajını bekle
        message = self.pg.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)

        # Mesajın alınıp alınmadığını kontrol et
        if message is None:
            PrintColours.print_error("İrtifa bilgisi alınamadı!")
            return None

        # İrtifayı metre cinsine çevir ve döndür
        alt = message.relative_alt / 1000.0  # milimetreden metreye çevir
        return alt

    # def update_heading(self):
    #     """Update the current heading of the pg at a fixed frequency."""
    #     while True:
    #         msg = self.master.recv_match(type='VFR_HUD', blocking=True)
    #         self.current_heading_g = msg.heading
