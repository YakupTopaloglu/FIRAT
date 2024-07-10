import cv2
import socket
import struct
import pickle
import numpy as np
from gtts import gTTS
import random
from playsound import playsound
import os
import threading
from ultralytics import YOLO


# Socket oluştur
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('localhost', 8000))

data = b""
payload_size = struct.calcsize("Q")

while True:
    while len(data) < payload_size:
        packet = client_socket.recv(4*1024)
        if not packet: break
        data += packet

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("Q", packed_msg_size)[0]

    while len(data) < msg_size:
        packet = client_socket.recv(4*1024)
        if not packet: break
        data += packet

    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = pickle.loads(frame_data)

    # Renk dönüşümü yerine direkt olarak GRAY formatında alalım
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    model=YOLO("yolov8n.pt")
    frame = pickle.loads(frame_data)

    # İmajı göstermek için renk formatını yeniden ayarlayabiliriz
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Veya BGR2GRAY olarak kalabilir, bağlı olarak
    cv2.imshow('Face Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

client_socket.close()
cv2.destroyAllWindows()
