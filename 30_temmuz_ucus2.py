import socket

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 9999))  # Raspberry Pi'nin IP'si ve port numarası
    server_socket.listen(1)
    print('Server is listening on port localhost')
    
    while True:
        client_socket, addr = server_socket.accept()
        print(f'Connection from {addr}')
        message = client_socket.recv(1024).decode('utf-8')
        print(f'Received message: {message}')
        client_socket.close()

if __name__ == '__main__':
    start_server()
    
