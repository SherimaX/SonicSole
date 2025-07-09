import socket
import time

print("Starting test script...", flush=True)

target_ip = "192.168.50.109"
target_port = 20000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Function to send a pressure value for a given duration
def send_pressure(pressure_value):
    data = pressure_value.to_bytes(2, byteorder='little')
    while True:
        sock.sendto(data, (target_ip, target_port))
        print(f"Sent pressure: {pressure_value} to {target_ip}:{target_port}", flush=True)
        time.sleep(0.001)
send_pressure(900)
print("✅ Done sending test pressures.", flush=True)
