import time
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

# Drone bağlantısı
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

def squareMission():
    commands = drone.commands
    commands.clear()

    commands.add(
        Command(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10
        )
    )
    
    commands.add(
        Command(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
            -35.36294226, 149.16517598, 10
        )
    )
    commands.add(
        Command(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
            -35.36289032, 149.16552597, 10
        )
    )
    commands.add(
        Command(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
            -35.36324584, 149.16559858, 10
        )
    )
    commands.add(
        Command(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
    )

    commands.upload()
    print("Komutlar yüklendi.")

takeoff(10)
time.sleep(3)
drone.mode = VehicleMode("AUTO")
squareMission()



while drone.mode.name != 'AUTO':
    print("AUTO moduna geçiliyor...")
    time.sleep(1)

while True:
    print(drone.velocity)
    nextWP = drone.commands.next
    time.sleep(1)