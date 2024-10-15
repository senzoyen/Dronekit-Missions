#gazebo --verbose worlds/iris_arducopter_runway.world
#cd ~/ardupilot/ArduCopter
#../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
import time
from dronekit import connect, VehicleMode,LocationGlobalRelative

drone = connect('127.0.0.1:14550', wait_ready= True)#sim çalıştığı port , bu porta bağlanana kadar tekrar dene

# print(f'Drone Arm Durumu {drone.armed}') 
# print (f'Global Frame{drone.location.global_frame}')#altitude kısmı burda yerden yükseklik
# print (f'Global Relative Frame{drone.location.global_relative_frame}')#altitude kısmı vurda deniz seviyesine göre yükseklik
# print(f'İrtifa:{drone.location.global_relative_frame.alt}')#alt alır
x = 10
def takeoff(altitude):
    
    while drone.is_armable:
        print("drone arm edilebilir")
        drone.mode = VehicleMode("GUIDED")
        
        print(str(drone.mode) + "moda alındı")
        drone.armed = True #arm durumu 1

        while drone.armed is not True:
            print("Drone arm ediliyor")

        print("drone arm edildi")
        drone.simple_takeoff(x)
        time.sleep(1)
        break
takeoff(x)
location = LocationGlobalRelative(-35.36223671, 149.16509335, 20)
altitude = drone.location.global_relative_frame.alt

while altitude <9.1:
    altitude = drone.location.global_relative_frame.alt
    print(altitude)
    if x - 1 < altitude < x + 1:
        print("alt {}".format(altitude))
        drone.simple_goto(location)