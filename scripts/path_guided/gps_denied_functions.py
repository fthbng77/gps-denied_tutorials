from pymavlink import mavutil
from PrintColours import PrintColours  # PrintColours sınıfını içe aktarın
import time
import math
import rospy
from geometry_msgs.msg import PoseStamped

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
            self.drone = None  # You can set it to None or some default value

        self.current_pose = None
        rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.camera_pose_callback)

    def camera_pose_callback(self, data):
        self.current_pose = data

    def arm(self):
        # Arm the drone
        PrintColours.print_info("Arming the drone")
        self.drone.mav.command_long_send(
            self.drone.target_system, self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    def takeoff(self, altitude=5):
        self.drone.motors_armed_wait()
        PrintColours.print_info("Drone is armed")
        
        # Set mode to GUIDED
        self.set_mode("GUIDED")
        PrintColours.print_info("Mode set to GUIDED")

        # Takeoff to the specified altitude (in meters)
        PrintColours.print_blinking_warning("Waiting for the drone to takeoff")
        time.sleep(2)
        PrintColours.print_info("Taking off")
        self.drone.mav.command_long_send(
            self.drone.target_system, self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
        )
        PrintColours.print_info("Takeoff command sent")

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

    def get_attitude(self):
        # Attitude mesajını bekleyin
        attitude_msg = self.drone.recv_match(type='ATTITUDE', blocking=True)
        # roll, pitch ve yaw değerlerini döndürün
        return attitude_msg.roll, attitude_msg.pitch, attitude_msg.yaw
    
    def move(self, x, y, z):
        """
        Dronu yerel koordinatlarda (x, y, z) konumuna hareket ettirir.
        """
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
        time.sleep(1)

    def move_relative(self, forward, right, down):
        # Drone'un başlangıçtaki roll, pitch, yaw değerlerini al
        roll, pitch, yaw = self.get_attitude()

        # Hareket vektörünü dronenin başlangıçtaki yönelimine göre döndür
        x = math.sin(yaw) * forward + math.cos(yaw) * right  # x ve y yer değiştirdi
        y = math.cos(yaw) * forward - math.sin(yaw) * right
        z = down  # Z ekseninin yönü değişti (yukarı yerine aşağıya değil)

        self.move(x, y, z) # mevcut move fonksiyonunu kullanarak hareket ettir

    def turn(self, angle):
        self.drone.mav.command_long_send(
            self.drone.target_system, self.drone.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, angle, 0, 0, 0, 0, 0, 0)
        time.sleep(1)

    def follow_orbslam_pose(self):
        if self.current_pose is not None:
            pos = self.current_pose.pose.position
            self.move(pos.x, pos.y, pos.z)
        else:
            PrintColours.print_warning("No ORB-SLAM pose data available.")

