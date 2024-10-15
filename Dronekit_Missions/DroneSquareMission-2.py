from dronekit import connect,VehicleMode
from pymavlink import mavutil
import time

drone = connect('127.0.0.1:14550', wait_ready=True)

def takeoff(altitude):
    while not drone.is_armable:
        print("Drone arm edilebilir durumda değil, bekleniyor...")
        time.sleep(1)

    print("Drone arm edilebilir, arm işlemi başlıyor...")
    drone.mode = VehicleMode("GUIDED")
    
    while not drone.mode.name == 'GUIDED':
        print("GUIDED moduna geçiliyor...")
        time.sleep(1)

    drone.armed = True
    while not drone.armed:
        print("Drone arm ediliyor...")
        time.sleep(1)

    print("Drone arm edildi, takeoff yapılıyor...")
    drone.simple_takeoff(altitude)

    while True:
        print(f"Mevcut Yükseklik: {drone.location.global_relative_frame.alt}")
        if drone.location.global_relative_frame.alt >= altitude * 0.95:
            print("Hedef yüksekliğe ulaşıldı!")
            break
        time.sleep(1)

# GLOBALİ REFERANS ALAN HAREKET FONKSİYONU
def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the drone fly to a specified
    location in the North, East, Down frame.
    """
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to drone
    drone.send_mavlink(msg)

def square():
    goto_position_target_local_ned(50, 0, -10)
    time.sleep(10)
    goto_position_target_local_ned(0, 50, -10)
    time.sleep(10)
    goto_position_target_local_ned(-50, 0, -10)
    time.sleep(10)
    goto_position_target_local_ned(0, -50, -10)
    

takeoff(10)
time.sleep(10)
square()




    