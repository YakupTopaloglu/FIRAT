from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil

firat = connect('/dev/ttyACM0',wait_ready=True, baud=115200)


# Gerekli ayarlamar.

# Safe Mode yarıçapı fonsiyonu.
def set_geofence_radius(vehicle, radius):
    # Define the home location (current location)
    home_location = vehicle.location.global_relative_frame

    # Set the parameters for the geofence
    vehicle.parameters['FENCE_ENABLE'] = 1  # Enable geofence
    vehicle.parameters['FENCE_TYPE'] = 3  # Circular geofence
    vehicle.parameters['FENCE_RADIUS'] = radius  # Radius in meters

    print(f"Geofence set with radius: {radius} meters")

# Görev radius değeri ayarlama foksiyonu.
def set_waypoint_radius(radius):
    firat.parameters['WP_RADIUS'] = radius

# Eve dönüş yarıçapı fonksiyonu.
def set_rtl_radius(radius):
    firat.parameters['RTL_RADIUS'] = radius


# RTL radius değeri
set_rtl_radius(2) # RTL yarıçapını 2 metre olarak ayarlama.

# Görev radius değeri
set_waypoint_radius(2) # Görev radius değerini 2 olarak ayarlama.

# Safe Mode yarıçapı.
set_geofence_radius(firat, 20)  # Bulunduğu noktadan 20 metre uzakta bulunursa RTL moduna girer. 


# Görev fonksiyonları.

def takeoff(irtifa):
    while firat.is_armable is not True:
        print("İHA arm edilebilir durumda değil.")
        time.sleep(1)


    print("İHA arm edilebilir.")

    firat.mode = VehicleMode("GUIDED")

    firat.armed = True
    firat.airspeed = 2

    while firat.armed is not True:
        print("İHA arm ediliyor...")
        time.sleep(0.5)

    print("İHA arm edildi.")

    firat.simple_takeoff(irtifa)
    
    while firat.location.global_relative_frame.alt < irtifa * 0.9:
        print("İha hedefe yükseliyor.")
        time.sleep(1)

def gorev_ekle():
    global komut
    komut = firat.commands

    komut.clear()
    time.sleep(1)
    firat.airspeed = 2

    # TAKEOFF
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 2))

    # WAYPOINT
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.1873927, 29.1287862, 2))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.1873208, 29.1288164, 2))

    # RTL
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    
    # DOĞRULAMA
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    komut.upload()
    print("Komutlar yükleniyor...")


takeoff(2) # Yerden 2 metre yükselmesini sağlar.

gorev_ekle() # Görevi başltır. 

komut.next = 0

firat.mode = VehicleMode("AUTO")

while True:
    next_waypoint = komut.next

    print(f"Sıradaki komut {next_waypoint}")
    time.sleep(1)

    if next_waypoint is 4:
        print("Görev bitti.")
        break

print("Döngüden çıkıldı.")

firat.mode = VehicleMode("LAND")

firat.close() #Bağlantı kesme.