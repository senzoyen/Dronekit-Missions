import time
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import math

drone = connect('127.0.0.1:14550', wait_ready=True)



wayPoints = [
    (-35.36294226, 149.16517598),
    (-35.36289032, 149.16552597),
    (-35.36324584, 149.16559858),
    (-35.36326200, 149.16523582)
]

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


def lenM(wp1, wp2):
  
    R = 6371.0  # Dünya'nın yarıçapı kilometre cinsinden

    lat1, lon1 = math.radians(wp1[0]), math.radians(wp1[1])
    lat2, lon2 = math.radians(wp2[0]), math.radians(wp2[1])

    # Enlem mesafesi
    dlat = lat2 - lat1
    enlem_mesafesi = R * dlat

    # Boylam mesafesi
    dlon = lon2 - lon1
    boylam_mesafesi = R * dlon * math.cos((lat1 + lat2) / 2)  # Enlemin ortalaması

    return enlem_mesafesi, boylam_mesafesi





def newLoc(start_lat, start_lon, kuzey_metre, dogu_metre):

    R = 6378137  # Dünya'nın yarıçapı (metre cinsinden)

    # Enlemde kuzey veya güney yönünde değişiklik
    new_lat = start_lat + (kuzey_metre / R) * (180 / math.pi)
    
    # Boylamda doğu veya batı yönünde değişiklik
    new_lon = start_lon + (dogu_metre / (R * math.cos(math.pi * start_lat / 180))) * (180 / math.pi)

    return (new_lat, new_lon)



enlem_mesafesi, boylam_mesafesi = lenM(wayPoints[0], wayPoints[1])
print(f"Enlem Mesafesi: {enlem_mesafesi * 1000:.2f} m")
print(f"Boylam Mesafesi: {boylam_mesafesi * 1000:.2f} m")
enlem_mesafesi_Metre = enlem_mesafesi * 1000
boylam_mesafesi_metre = boylam_mesafesi * 1000

i = int(enlem_mesafesi_Metre/2)
print(i)
lat,lon = (-35.36289032, 149.16552597)

takeoff(10)
time.sleep(15)

while i >= 1:
    lat, lon = newLoc(lat, lon, -2, 0)
    location = str(lat) +","+ str(lon)
    print(f"Yeni konum: Enlem: {lat:.8f}, Boylam: {lon:.8f}")
    drone.simple_goto(location)
    time.sleep(10)
    if i % 2 == 0:
        lat, lon = newLoc(lat, lon, 0, boylam_mesafesi_metre)
    else:
        lat, lon = newLoc(lat, lon, 0, -boylam_mesafesi_metre)
    location = str(lat) +","+ str(lon)
    drone.simple_goto(location)
    time.sleep(10)
    print(f"Yeni konum: Enlem: {lat:.8f}, Boylam: {lon:.8f}")
    i -= 1








