import socket
import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
from gtts import gTTS
import random
import os
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class SesliAsistan:
    def __init__(self):
        Gst.init(None)

    def seslendirme(self, metin):
        xtts = gTTS(text=metin, lang="tr")
        dosya = "dosya" + str(random.randint(0, 123)) + ".mp3"
        xtts.save(dosya)
        
        # Play the audio file using GStreamer
        player = Gst.ElementFactory.make("playbin", "player")
        player.set_property("uri", f"file://{os.path.abspath(dosya)}")
        player.set_state(Gst.State.PLAYING)
        
        # Wait for the audio to finish playing
        bus = player.get_bus()
        msg = bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
        
        # Clean up
        player.set_state(Gst.State.NULL)
        os.remove(dosya)

def start_server(vehicle):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('192.168.137.180', 9999))  # Raspberry Pi'nin IP'si ve port numarası
    server_socket.listen(1)
    print('Server is listening on port 9999')
    
    while True:
        client_socket, addr = server_socket.accept()
        print(f'Connection from {addr}')
        message = client_socket.recv(1024).decode('utf-8')
        print(f'Received message: {message}')
        if message == '15':  # wp numarasını değiştir
            # Set mode to GUIDED
            vehicle.mode = VehicleMode("GUIDED")
            # Set the target location in global-relative frame
            a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)  # buradaki değerler değişiyor
            vehicle.simple_goto(a_location)
            print("Sent vehicle to new location")
            
            time.sleep(6)
            
            # Start the voice assistant
            sesli_asistan = SesliAsistan()
            
            end_time = time.time() + 5  #  saniye boyunca döngü
            while time.time() < end_time:
                sesli_asistan.seslendirme("Yeni bir konuma gitmekteyiz.")
                time.sleep(5)  # 5 saniye arayla sesli mesajlar gönder
                
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

if __name__ == '__main__':
    vehicle = connectMyPlane()
    
    try:
        start_server(vehicle)
    except KeyboardInterrupt:
        print("User interrupted, exiting...")
    finally:
        vehicle.close()
