import socket
import struct
import pickle
import cv2
import threading

# Sunucu bilgileri
HOST = '10.15.160.171'  # Sunucu bilgisayarın yerel IP adresi (Örneğin)
PORT = 8000

# Socket oluşturma
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))
print('Sunucuya bağlanıldı.')

data = b""
payload_size = struct.calcsize("L")

def receive_frames():
    global data

    while True:
        while len(data) < payload_size:
            packet = client_socket.recv(4 * 1024)
            if not packet:
                break
            data += packet

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("L", packed_msg_size)[0]

        while len(data) < msg_size:
            packet = client_socket.recv(4 * 1024)
            if not packet:
                break
            data += packet

        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data)

        cv2.imshow('Istemci - Alınan Goruntu', frame)
        if cv2.waitKey(1) == 27:  # ESC tuşuna basıldığında çık
            break

def close_resources():
    client_socket.close()
    cv2.destroyAllWindows()

# Alım thread'i başlatma
receive_thread = threading.Thread(target=receive_frames)
receive_thread.start()

# Thread'in bitmesini bekleme
receive_thread.join()

# Kaynakları serbest bırakma
close_resources()
