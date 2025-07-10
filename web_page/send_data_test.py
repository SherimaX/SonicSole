'''import socket
import time
import struct

print("Starting test script...", flush=True)

target_ip = "192.168.50.109"
target_port = 21000  # Unified port you're using in Flask

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Function to send test data (5 floats: fore, heel, ax, ay, az)
def send_combined_test_data(fore, heel, ax, ay, az):
    while True:
        # Pack data as 5 floats (little-endian by default)
        data = struct.pack('5f', fore, heel, ax, ay, az)
        sock.sendto(data, (target_ip, target_port))
        print(f"Sent -> Fore: {fore}, Heel: {heel}, ax: {ax}, ay: {ay}, az: {az}", flush=True)
        time.sleep(0.01)  # Send at 100Hz (adjust as needed)

# Example: Fore=900, Heel=850, Simulated accel values
send_combined_test_data(900.0, 850.0, 0.02, -0.01, 9.81)

print("✅ Done sending test packets.", flush=True)
'''

import socket
import time
import struct
import math

print("Starting jump test simulation...", flush=True)

target_ip = "192.168.50.109"
target_port = 21000 

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_packet(fore, heel, ax, ay, az):
    data = struct.pack('5f', fore, heel, ax, ay, az)
    sock.sendto(data, (target_ip, target_port))

# Step 1: Standing still (10 seconds)
for _ in range(10):
    send_packet(900, 900, 0.0, 0.0, 0)

    time.sleep(0.01)

# Step 2: Takeoff 
for i in range(45):
    send_packet(0, 0, 0.0, 0.0, 9.8)  # Jump upward burst
    time.sleep(0.01)

# Step 3: in air (1 seconds)
#for _ in range(100):
 #   send_packet(0, 0, 0.0, 0.0, 0.0)  # Near free-fall
  #  time.sleep(0.01)

# Step 4: Landing 
for _ in range(10):
    send_packet(900, 900, 0.0, 0.0, 0)  # Downward impact spike
    time.sleep(0.01)

# Step 5: Back to standing
for _ in range(10):
    send_packet(900, 900, 0.0, 0.0, 0)
    time.sleep(0.01)

print("✅ Simulated jump complete.", flush=True)
