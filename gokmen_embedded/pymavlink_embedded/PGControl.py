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


# ------- Uçuş Komutları ------- 

    def goto_gps(self, lat, lon, alt, seq=0, wait_for_ack=True):
        """
        Dronu belirtilen GPS koordinatlarına (enlem, boylam, irtifa) hareket ettirir.

        Parameters:
        lat (float): Hedef enlem.
        lon (float): Hedef boylam.
        alt (float): Hedef irtifa.
        seq (int): Görev dizisi numarası. Varsayılan 0.
        wait_for_ack (bool): Komutun kabul edilip edilmediğini bekler. Varsayılan True.
        """
        PrintColours.print_info(f"Going to GPS: {lat}, {lon}, {alt}")

        # Görev öğesini (waypoint) gönder
        self.pg.mav.mission_item_send(
            self.pg.target_system,  # hedef sistem
            self.pg.target_component,  # hedef bileşen
            seq,  # görev dizisi numarası
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # çerçeve
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # komut
            2,  # current (2 ise bu waypoint şu anki)
            0,  # auto continue (otomatik devam)
            0, 0, 0,  # kullanılmayan parametreler
            lat,  # hedef enlem
            lon,  # hedef boylam
            alt,  # hedef irtifa
            0     # MAVLink 2.0 için ekstra parametre (kullanılmıyor)
        )

        if wait_for_ack:
            # Görev onayının beklenmesi
            ack_msg = self.pg.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if ack_msg and ack_msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                PrintColours.print_info("Waypoint acknowledged by the pg")
            else:
                PrintColours.print_error("Failed to receive waypoint acknowledgment")

    def move_with_velocity(self, vx, vy, vz, duration):
        """
        PG'yi belirli bir hızla belirli bir süre boyunca hareket ettirir.
        
        Parameters:
        vx (float): Kuzey yönündeki hız (m/s).
        vy (float): Doğu yönündeki hız (m/s).
        vz (float): Aşağı yönündeki hız (m/s).
        duration (float): Hareket süresi (saniye cinsinden).
        """
        PrintColours.print_info(f"Moving with velocity vx: {vx}, vy: {vy}, vz: {vz} for {duration} seconds")

        # MAVLink komutunu gönder
        self.pg.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0,  # zaman damgası (genellikle 0 yapılır)
                self.pg.target_system,  # hedef sistem
                self.pg.target_component,  # hedef bileşen
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # referans çerçeve (NED çerçevesi)
                0b0000111111000110,  # tip maskesi (sadece hız vektörlerini belirtiyoruz)
                0, 0, 0,  # x, y, z pozisyonu (kullanılmıyor)
                vx, vy, vz,  # x, y, z hızları
                0, 0, 0,  # açısal hızlar (roll, pitch, yaw hızları)
                0, 0, 0  # afz, yaw, yaw_rate (eklenmesi gereken parametreler)
            )
        )

        # Hareket süresi boyunca bekleme
        time.sleep(duration)

        # Hareketi durdurmak için hızları sıfırla
        self.pg.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, 
                self.pg.target_system, 
                self.pg.target_component, 
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                0b0000111111000110, 
                0, 0, 0, 
                0, 0, 0,
                0, 0, 0  # afz, yaw, yaw_rate (sıfırlarız)
            )
        )

    def move_enu_with_speed(self, east, north, up, speed):
        """
        ENU (East-North-Up) yönünde belirli bir hızla hareket ettirir.
        
        Parameters:
        east (float): Doğu yönündeki uzaklık (metre cinsinden).
        north (float): Kuzey yönündeki uzaklık (metre cinsinden).
        up (float): Yukarı yönündeki yükseklik değişikliği (metre cinsinden).
        speed (float): Hız (m/s cinsinden).
        """
        # Uzaklığa göre sürenin hesaplanması
        distance = (east**2 + north**2 + up**2)**0.5
        duration = distance / speed  # Süre = Mesafe / Hız

        # Kuzey, Doğu, Yukarı hızları belirleme
        vx = north / duration  # Kuzey yönünde hız
        vy = east / duration   # Doğu yönünde hız
        vz = -up / duration    # Yukarı yönündeki hız (NED pozitif aşağıya)

        # Hareket fonksiyonunu çağır
        self.move_with_velocity(vx, vy, vz, duration)

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


# ------- Destekleyici Kontroller ------- 

    def condition_yaw(self, heading, relative=False, cw=1):
        """
        pg'un yaw (sapma) açısını belirler.

        Args:
        heading (float): İstenen yaw açısı (derece cinsinden).
        relative (bool): True ise açı seyahat yönüne göre ayarlanır, False ise mutlak açı kullanılır.
        cw (int): Dönüş yönü. 1 saat yönünde (cw), -1 saat yönünün tersinde (ccw).
        """

        # Yön belirleme: relative true ise yön seyahat yönüne göre, false ise mutlak açıdır
        is_relative = 1 if relative else 0

        # MAVLink komutunu gönder
        self.pg.mav.command_long_send(
            self.pg.target_system, self.pg.target_component,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # komut: Yaw ayarla
            0,  # confirmation
            heading,  # param 1: yaw açısı (derece)
            0,  # param 2: yaw hızı (derece/saniye) - 0 ise varsayılan hız kullanılır
            cw,  # param 3: dönüş yönü (-1 ccw, 1 cw)
            is_relative,  # param 4: relatif mi, mutlak mı
            0, 0, 0  # diğer parametreler kullanılmıyor
        )

        # Yaw ayarlandıktan sonra bir gecikme ekleyebilirsiniz
        PrintColours.print_info(f"Yaw ayarlandı: Başlık {heading} derece, Göreceli: {relative}, Dönüş yönü: {'cw' if cw == 1 else 'ccw'}")

    def set_speed(self, speed, wait_for_ack=True):
        """
        Dronun hızını ayarlar (m/s cinsinden).

        Parameters:
        speed (float): Hedef hız (m/s).
        wait_for_ack (bool): Hız parametresinin kabul edilip edilmediğini bekler. Varsayılan True.
        """
        PrintColours.print_info(f"Setting speed to {speed} m/s")
        
        # Hız parametresini gönder
        self.pg.mav.param_set_send(
            self.pg.target_system, 
            self.pg.target_component,
            b"WPNAV_SPEED",  # Hız parametresi
            speed,  # Hedef hız
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Veri tipi
        )
        
        if wait_for_ack:
            # Hız parametresinin ayarlandığını doğrulamak için onay mesajını bekle
            ack_msg = self.pg.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if ack_msg and ack_msg.param_id == b"WPNAV_SPEED" and abs(ack_msg.param_value - speed) < 0.01:
                PrintColours.print_info("Speed set successfully")
            else:
                PrintColours.print_error("Failed to set speed")

# ------- Durum ve Pozisyon Bilgileri ------- 

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


    # def update_heading(self):
    #     """Update the current heading of the pg at a fixed frequency."""
    #     while True:
    #         msg = self.master.recv_match(type='VFR_HUD', blocking=True)
    #         self.current_heading_g = msg.heading