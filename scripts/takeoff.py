from gps_denied_functions2 import PymavlinkFunctions
import time


def main():
    connection_string = "udpin:127.0.0.1:14550" 
    drone = PymavlinkFunctions(connection_string)

    drone.arm()
    time.sleep(2)
    drone.takeoff(10)  # Take off to an altitude of 10 meters
    time.sleep(5)
    
    # Land after drawing the square
    drone.land()
    time.sleep(5)
    drone.disarm()
    
    print("Mission completed!")

if __name__ == "__main__":
    main()
