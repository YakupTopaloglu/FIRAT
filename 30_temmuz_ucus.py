import socket
from dronekit import connect, VehicleMode
import time
import argparse

def send_message_to_rpi():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rpi_ip = 'localhost'  # Raspberry Pi'nin IP adresi
    rpi_port = 9999  # Raspberry Pi üzerinde sunucu kodunda belirtilen port numarası
    
    client_socket.connect((rpi_ip, rpi_port))
    message = 'Waypoint 15 reached'
    client_socket.sendall(message.encode('utf-8'))
    client_socket.close()

def connectMyPlane():
    # Simülasyonun bağlantısı
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600

    print(f'Connecting to vehicle on: {connection_string}')
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

def arm_and_takeoff(vehicle, target_altitude):
    """
    Arms vehicle and fly to a target altitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("AUTO")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before switching to AUTO mode
    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    vehicle.mode = VehicleMode("AUTO")

def monitor_waypoints(vehicle):
    print("Monitoring waypoints...")
    while True:
        next_waypoint = vehicle.commands.next
        print(f'Current waypoint: {next_waypoint}')
        
        if next_waypoint == 15:
            send_message_to_rpi()
            print('Message sent to Raspberry Pi')
            break
        
        time.sleep(1)

if __name__ == '__main__':
    vehicle = connectMyPlane()
    
    try:
        arm_and_takeoff(vehicle, 10)  # Aracı ARM yap ve 10 metreye yüksel
        monitor_waypoints(vehicle)
    except KeyboardInterrupt:
        print("User interrupted, exiting...")
    finally:
        vehicle.close()