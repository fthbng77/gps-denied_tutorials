from pymavlink import mavutil
from PrintColours import PrintColours  # PrintColours sınıfını içe aktarın
import time
import math

class PymavlinkFunctions:
    def __init__(self, connection_string=None):
        # Connect to the drone only if connection_string is provided
        if connection_string is not None:
            self.drone = mavutil.mavlink_connection(connection_string)
            # Wait for the drone to connect
            msg = self.drone.wait_heartbeat()
            PrintColours.print_info("Heartbeat from system (system %u component %u)" %
                                    (self.drone.target_system, self.drone.target_component))
        else:
            self.drone = None

    def arm(self):
        # Arm the drone
        PrintColours.print_info("Arming the drone")
        self.drone.mav.command_long_send(
            self.drone.target_system, self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )

    def takeoff(self, altitude=5):
        PrintColours.print_blinking_warning("Preparing for takeoff")
        current_altitude = 0
        step_size = 0.25  # Altitude to increase in each step in meters
        while current_altitude < altitude:
            current_altitude += step_size
            if current_altitude > altitude:
                current_altitude = altitude
            PrintColours.print_info(f"Taking off to altitude {current_altitude} meters")
            self.drone.mav.command_long_send(
                self.drone.target_system, self.drone.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, current_altitude
            )
            time.sleep(2) 

    def send_command(self, forward, right, down):
        if self.drone is None:
            print("No connection to send commands.")
            return
        self.drone.mav.set_position_target_local_ned_send(
            0,
            1,
            1,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            forward, right, down,
            0, 0, 0,
            0, 0
        )

    def condition_yaw(self, speed, relative, direction):
        if self.drone is None:
            print("No connection to send commands.")
            return
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            speed,
            0,
            direction,
            relative,
            0, 0, 0
        )

    def set_forward_velocity(self, velocity):
        """
        Drona ileri doğru bir hız vektörü gönderir.
        :param velocity: İleri hız (m/s)
        """
        # MAV_CMD_SET_POSITION_TARGET_LOCAL_NED kullanarak hız komutu gönder
        msg = self.drone.mav.set_position_target_local_ned_encode(
            0,       # time_boot_ms (timestamp)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (sadece hızı güncelle)
            0, 0, 0,  # x, y, z positions (ignored)
            velocity, 0, 0,  # X, Y, Z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)     # yaw, yaw_rate (not used)
        self.drone.send_mavlink(msg)

    def set_speed(self, speed):
        # Set the drone's speed (in m/s)
        PrintColours.print_info(f"Setting speed to {speed} m/s")
        self.drone.mav.param_set_send(
            self.drone.target_system, self.drone.target_component,
            b"WPNAV_SPEED", speed, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def set_mode(self, mode):
        # Set the drone's flight mode
        PrintColours.print_info(f"Setting mode to {mode}")
        mode_mapping = self.drone.mode_mapping()
        mode_id = mode_mapping.get(mode)
        if mode_id is not None:
            self.drone.mav.set_mode_send(
                self.drone.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
        else:
            PrintColours.print_warning(f"Unknown mode: {mode}")

    def land(self):
        # Land the drone
        PrintColours.print_info("Landing")
        
        # LAND modu için MAVLink kodu 9'dur
        LAND_MODE = 9
        
        # Set the mode to LAND using a MAVLink command
        self.drone.mav.set_mode_send(
            self.drone.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            LAND_MODE
        )
        time.sleep(2)

    def disarm(self):
        # Disarm the drone
        PrintColours.print_blinking_warning("Waiting for the drone to disarming")
        time.sleep(3)
        PrintColours.print_info("Disarming the drone")
        self.drone.mav.command_long_send(
            self.drone.target_system, self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )


    def get_position(self):
            """
            Gets the current global position of the drone.
            
            Returns:
            tuple: A tuple containing the latitude, longitude, and altitude of the drone.
            """
            # Get the raw message (this may be different depending on the mavlink version/library)
            # Make sure that you have established a mavlink connection and drone is communicating
            msg = self.drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            
            # Extract latitude, longitude, and altitude from the message
            latitude = msg.lat / 1e7  # degrees
            longitude = msg.lon / 1e7  # degrees
            altitude = msg.alt / 1e3  # meters
            
            return latitude, longitude, altitude

    def check_waypoint_reached(self, waypoint, tolerance=1):
        """
        Check if the drone reached the waypoint within a certain tolerance.

        Parameters:
        waypoint (list): The target waypoint [x, y, z]
        tolerance (float): The tolerance within which the drone is considered to have reached the waypoint. Default is 1 meter.

        Returns:
        bool: True if the drone reached the waypoint, False otherwise.
        """
        current_position = self.get_position()  # Assuming get_position returns a list [x, y, z]
        
        distance = ((current_position[0] - waypoint[0])**2 + 
                    (current_position[1] - waypoint[1])**2 + 
                    (current_position[2] - waypoint[2])**2)**0.5
        
        if distance <= tolerance:
            return True
        else:
            return False
        
    def condition_yaw(self, heading, relative=False, cw=0):
        """
        Sets the drone's yaw condition.

        Args:
        heading (float): The desired yaw angle in degrees.
        relative (bool): If True, the heading is relative to the travel direction. If False, the heading is an absolute angle.
        cw (int): Direction of rotation. 0 for counter-clockwise (ccw), 1 for clockwise (cw).
        """

        if relative:
            is_relative = 1  # Yön, seyahat yönüne göre belirlenir
        else:
            is_relative = 0  # Yön, mutlak bir açı olarak belirlenir

        self.drone.mav.command_long_send(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            cw,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used

    def get_attitude(self):
        # Attitude mesajını bekleyin
        attitude_msg = self.drone.recv_match(type='ATTITUDE', blocking=True)
        # roll, pitch ve yaw değerlerini döndürün
        return attitude_msg.roll, attitude_msg.pitch, attitude_msg.yaw
    
    def move(self, x, y, z):
        """
        Dronu yerel koordinatlarda (x, y, z) konumuna hareket ettirir.
        """
        # Burada self.drone'in mavutil ile açılmış bir bağlantı olduğunu varsayıyorum.
        # Eğer bu bağlantı farklı bir şekilde adlandırılmışsa, uygun adı kullanın.
        self.drone.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0,  # zaman
                self.drone.target_system,  # hedef sistem
                self.drone.target_component,  # hedef bileşen
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # referans çerçeve
                0b0000111111000111,  # tip maskesi
                0, 0, 0,  # Vx, Vy, Vz
                x, y, z,  # x, y, z pozisyonu
                0, 0, 0, 0, 0  # diğer parametreler
            )
        )
        # hareket emri verildikten sonra 1 saniye bekle
        time.sleep(1)

# move fonksiyonunu kullanarak dronenun yerel koordinatlarda,
# move_relative fonksiyonu dronenun göreceli koordinatlarda hareket etmesini sağlar 

    def move_relative(self, forward, right, down):
        # Drone'un başlangıçtaki roll, pitch, yaw değerlerini al
        roll, pitch, yaw = self.get_attitude()

        # Hareket vektörünü dronenin başlangıçtaki yönelimine göre döndür
        x = math.sin(yaw) * forward + math.cos(yaw) * right  # x ve y yer değiştirdi
        y = math.cos(yaw) * forward - math.sin(yaw) * right
        z = down  # Z ekseninin yönü değişti (yukarı yerine aşağıya değil)
        print(f"Moving to x:{x}, y:{y}, z:{z}")
        self.move(x, y, z) # mevcut move fonksiyonunu kullanarak hareket ettir

    
    def turn(self, angle):
        self.drone.mav.command_long_send(
            self.drone.target_system, self.drone.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, angle, 0, 0, 0, 0, 0, 0)
        time.sleep(1)
    
    def goto_gps(self, lat, lon, alt):
        """
        Dronu belirtilen GPS koordinatlarına (enlem, boylam, irtifa) hareket ettirir.
        """
        PrintColours.print_info(f"Going to GPS: {lat}, {lon}, {alt}")

        self.drone.mav.mission_item_send(
            self.drone.target_system,  # target system
            self.drone.target_component,  # target component
            0,  # sequence number
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
            2,  # current
            0,  # auto continue
            0, 0, 0,  # params 1-3
            lat,  # latitude
            lon,  # longitude
            alt,  # altitude
            0     # z parameter for MAVLink 2.0, you can set it to 0
        )


    def update_heading(self):
        """Update the current heading of the drone at a fixed frequency."""
        while True:
            msg = self.drone.recv_match(type='VFR_HUD', blocking=True)
            self.current_heading_g = msg.heading

    def get_current_heading(self):
        """Returns the current heading of the drone.

        Returns:
            Heading (Float): θ in is degrees.
        """
        return self.current_heading_g
