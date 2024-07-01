import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 20000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet, UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
    
    # Convert the received data to an integer (assuming big-endian byte order)
    try:
        received_int = int.from_bytes(data, byteorder='big')
        print(f"Received message: {received_int}")

    except Exception as e:
        print(f"Error decoding message: {e}")
