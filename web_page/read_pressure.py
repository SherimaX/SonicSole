import socket

UDP_IP = "127.0.0.1"
UDP_PORT1 = 20000
UDP_PORT2 = 21000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet, UDP
sock.bind((UDP_IP, UDP_PORT1))

sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2.bind((UDP_IP, UDP_PORT2))

while True:
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
    data2, addr2 = sock.recvfrom(1024)
    
    try:
        received_int = int.from_bytes(data, byteorder='little')
        print(f"Received message: {received_int}")
        recieved_int2 = int.from_bytes(data2, byteorder='little')
        print(f"Received message: {recieved_int2}")


    except Exception as e:
        print(f"Error decoding message: {e}")
