import socket

UDP_IP = "0.0.0.0"

UDP_PORT = 21000      # Change if needed

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"Received from {addr}: {data.decode(errors='ignore')}")
except KeyboardInterrupt:
    print("\nStopped.")
    sock.close()
