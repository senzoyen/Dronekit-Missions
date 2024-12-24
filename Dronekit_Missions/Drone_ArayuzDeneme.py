import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import serial
import math
from math import copysign
import numpy as np

arduino_serial = serial.Serial('/dev/ttyACM2', 9600, timeout=1)

vehicle = connect('127.0.0.1:14550', baud=115200, wait_ready=True)
   
def send_startup_message():
    arduino_serial.write(b'Startup: Drone system initialized\n')

def set_mode(mode):
    vehicle.mode = VehicleMode(mode)
    while vehicle.mode.name != mode:
        print("Waiting for mode change ...")
        time.sleep(1)
    print("Mode changed to {}".format(mode))

def arm_drone():
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    vehicle.armed = True
    
    sayac = 0
    while not vehicle.armed:
        if sayac > 7:
            break
        print("Waiting for arming...")
        time.sleep(1)
    print("Drone armed")

def disarm_drone():
    vehicle.armed = False
    sayac = 0
    while vehicle.armed:
        if sayac > 7:
            break
        print("Waiting for disarming...")
        time.sleep(1)
    print("Drone disarmed")

def takeoff(altitude):
    arm_drone()
    print("Taking off to {} meters".format(altitude))
    vehicle.simple_takeoff(altitude)
    while True:
        print("Altitude: {}".format(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def land():
    print("Landing initiated")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode.name != "LAND":
        print("Waiting for mode change ...")
        time.sleep(1)
    print("Drone is landing")

def parse_coordinates(data):
    """Arduino'dan gelen koordinatları parse et."""
    # Gelen veriyi temizle, boşlukları ve fazladan karakterleri kaldır
    data = data.replace(' ', '')
    coordinates = data.split(',')

    # Koordinatlar çiftler halinde olacak, her bir çift (latitude, longitude)
    parsed_coordinates = []
    if len(coordinates) % 2 != 0:
        print(f"Geçersiz veri formatı: Koordinatlar çift olmalı. Alınan veri: {data}")
        return []

    for i in range(0, len(coordinates), 2):
        try:
            lat = float(coordinates[i])
            lon = float(coordinates[i + 1])
            parsed_coordinates.append((lat, lon))
        except ValueError:
            print(f"Geçersiz koordinat verisi: {coordinates[i]}, {coordinates[i + 1]}")
            continue

    return parsed_coordinates

def sort_coordinates(coordinates):
    """
    Gelen koordinatları sol üst, sağ üst, sol alt ve sağ alt şeklinde sıralar.
    """
    # Kuzey (en yüksek enlem) ve güney (en düşük enlem) noktalarını ayır
    coordinates = sorted(coordinates, key=lambda x: (-x[0], x[1]))  # Enlem yüksekten küçüğe, boylam küçükten büyüğe

    # İlk iki nokta (kuzey noktaları): Sol üst ve sağ üst  M  
    
    north = coordinates[:2]
    north = sorted(north, key=lambda x: x[1])  # Boylama göre sıralama (soldan sağa)

    # Son iki nokta (güney noktaları): Sol alt ve sağ alt
    south = coordinates[2:]
    south = sorted(south, key=lambda x: x[1])  # Boylama göre sıralama (soldan sağa)

    # Sonuç: [sol üst, sağ üst, sol alt, sağ alt]
    return [north[0], north[1], south[0], south[1]]

def haversine(lat1, lon1, lat2, lon2):
   
    R = 6371000 

    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Enlem ve boylam farklarını hesapla
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine formülünü uygula
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c  
    return distance

def Kare():
   takeoff(10)
   time.sleep(5)
   distance = 10
   directions = ['FORWARD', 'RIGHT', 'BACKWARD', 'LEFT']
   for direction in directions:
        move_to_new_location(vehicle,direction,distance)
   
def move_to_new_location(vehicle, direction, distance):
    """
    Drone'u belirtilen yön ve mesafeye göre hareket ettirir.
    
    :param vehicle: Drone aracı (DroneKit Vehicle nesnesi)
    :param direction: Hareket yönü ('FORWARD', 'BACKWARD', 'RIGHT', 'LEFT')
    :param distance: Hareket mesafesi (metre cinsinden)
    """
    # Mevcut konumu al
    current_location = vehicle.location.global_relative_frame
    current_lat = current_location.lat
    current_lon = current_location.lon
    current_alt = current_location.alt

    # Yeni koordinatları hesapla
    earth_radius = 6378137.0
    if direction.upper() == 'FORWARD':  # Kuzeye doğru
        d_lat = distance / earth_radius
        d_lon = 0
    elif direction.upper() == 'BACKWARD':  # Güneye doğru
        d_lat = -distance / earth_radius
        d_lon = 0
    elif direction.upper() == 'RIGHT':  # Doğuya doğru
        d_lat = 0
        d_lon = distance / (earth_radius * math.cos(math.radians(current_lat)))
    elif direction.upper() == 'LEFT':  # Batıya doğru
        d_lat = 0
        d_lon = -distance / (earth_radius * math.cos(math.radians(current_lat)))
    else:
        raise ValueError("Geçersiz yön! Lütfen 'FORWARD', 'BACKWARD', 'RIGHT' veya 'LEFT' kullanın.")
    
    new_lat = current_lat + math.degrees(d_lat)
    new_lon = current_lon + math.degrees(d_lon)

   
    target_location = LocationGlobalRelative(new_lat, new_lon, current_alt)
  
    print(f"{direction.upper()} yönünde {distance} metre hareket ediliyor.")
    print(f"Yeni hedef koordinatlar: Enlem: {new_lat}, Boylam: {new_lon}")
    goto_location(target_location)


def goto_location(target_location): 
    vehicle.simple_goto(target_location)
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = haversine(current_location.lat, current_location.lon, 
                             target_location.lat, target_location.lon)
        if distance < 1.0:  # 1 metre yakınlıkta dur
            print("Hedefe ulaşıldı.")
            break
        time.sleep(1)

def taramaa():
    """
    Tarama işlemini verilen dört koordinat arasında gerçekleştirir.
    """
    soor = [(-35.36172798, 149.16511112), (-35.39171954, 149.16623871), 
            (-35.36263066, 149.16511112), (-35.36262223, 149.166207670)]
    distance = 20  
    takeoff(10)

  
    start_location = LocationGlobalRelative(soor[0][0], soor[0][1], 10)
    goto_location(start_location)
    
    # Tarama örüntüsü
    max_steps_horizontal = int(haversine(soor[0][0], soor[0][1], soor[1][0], soor[1][1]) / distance)
    max_steps_vertical = int(haversine(soor[0][0], soor[0][1], soor[2][0], soor[2][1]))

    # Tarama örüntüsü
    for step in range(max_steps_horizontal):  
        move_to_new_location(vehicle, "RIGHT", distance)

        if step < max_steps_horizontal - 1:  
            if step % 2 == 0:  
                move_to_new_location(vehicle, "BACKWARD", max_steps_vertical)
            else:  
                move_to_new_location(vehicle, "FORWARD", max_steps_vertical)

    print("Tarama tamamlandı.")


def Tarama():
    # Tarama noktaları
    soor = [(-35.36300232, 149.16503176), (-35.36299981, 149.16547309), 
            (-35.36326692, 149.16503483), (-35.36326692, 149.16547155)]
    markers = []
    baslangic = LocationGlobalRelative(soor[0][0], soor[0][1], 10)  # Başlangıç konumu

    # Enlem ve boylam mesafelerini hesapla
    boylam = haversine(soor[0][0], soor[0][1], soor[2][0], soor[2][1])
    enlem = haversine(soor[0][0], soor[0][1], soor[1][0], soor[1][1])

    # Adım boyutu ve tekrar sayısı
    distance = 5  # Adım boyutu (metre)
    tekrarS = int(enlem / distance)

    print(f"Enlem mesafesi: {enlem:.2f} m")
    print(f"Boylam mesafesi: {boylam:.2f} m")
    print(f"Tekrar sayısı: {tekrarS}")

    # Tarama işlemi
    i = True
    for _ in range(tekrarS):
        # saga hareket
        markerR = move_to_new_location(vehicle, 'RIGHT', distance)
        markers.append(markerR)

        if i:
            # Geriye hareket
            markerB = move_to_new_location(vehicle, 'BACKWARD', boylam)
            markers.append(markerB)
            i = False
        else:
            # İleri hareket
            markerF = move_to_new_location(vehicle, 'FORWARD', boylam)
            markers.append(markerF)
            i = True

    # Kalkış
    takeoff(10)
    vehicle.simple_goto(baslangic)
    time.sleep(25)

    # Tüm noktaları dolaş
    for marker in markers:
        print(f"Hedef: {marker.lat}, {marker.lon}")
        vehicle.simple_goto(marker)
        time.sleep(15)


def set_mode(mode):
    vehicle.mode = VehicleMode(mode)       
    while vehicle.mode.name != mode:
        print("Waiting for mode change ...")
        time.sleep(1)
    print("Mode changed to {}".format(mode))

# def send_drone_data():
#     """
#     Drone verilerini seri port üzerinden Arduino'ya gönderir.
#     """
#     # Drone verilerini al
#     latitude = vehicle.location.global_relative_frame.lat
#     longitude = vehicle.location.global_relative_frame.lon
#     altitude = vehicle.location.global_relative_frame.alt
#     speed = vehicle.groundspeed
#     battery = vehicle.battery.level
#     slope = 0.0  # Placeholder

#         # Veriyi formatla
#     data = f"LAT:{latitude:.6f},LON:{longitude:.6f},ALT:{altitude:.2f},SPD:{speed:.2f},SLP:{slope:.2f},BAT:{battery}%\n"

#        # Arduino'ya gönder
#     arduino_serial.write(data.encode('utf-8'))
#            # Konsola yazdır
#     print(f"Gönderilen veri: {data.strip()}")
    

try:
    print("Arduino bağlantısı kuruldu. Veri bekleniyor...\n")
    mode = None  # Mode değişkenini başlangıçta tanımlayın

    while True:
        if arduino_serial.in_waiting > 0:
            raw_data = arduino_serial.readline()
            try:
                data = raw_data.decode('utf-8').strip()  
            except UnicodeDecodeError:
                data = raw_data.decode('ISO-8859-1').strip()

            print(f"Gelen veri: {data}")
            
            if data in ['STABILIZE', 'GUIDED', 'ALT_HOLD', 'LOITER','RTL']:
                set_mode(data)
            if data == 'ARM':
                arm_drone()
            if data == 'DISARM':
                disarm_drone()
            if data == 'TAKEOFF':
                takeoff(10)
            if data == 'LAND':
                land()
            if data == 'KARE':
                Kare()
            if data == 'Tarama':
                taramaa()
            # if mode in ['FORWARD', 'BACKWARD', 'UP', 'DOWN', 'RIGHT', 'LEFT']:
            #     send_movement_command(komut, distance)

            # Koordinat kontrolü
            elif ',' in data and all(c.isdigit() or c in '.-,' for c in data):
                coordinates = parse_coordinates(data)
                sortedCoord = sort_coordinates(coordinates)
                if sortedCoord:
                    print(f"soorted koordinatlar: {sortedCoord}")
            
            # else :
            #      send_drone_data()
        time.sleep(3)

except serial.SerialException as e:
    print(f"Seri bağlantı hatası: {e}")
except KeyboardInterrupt:
    print("\nProgram sonlandırılıyor...")
finally:
    if arduino_serial.is_open:
        arduino_serial.close()
        print("Arduino bağlantısı kapatıldı.")

