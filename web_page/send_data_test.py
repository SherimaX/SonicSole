import socket
import time
import struct

print("Starting test script...", flush=True)

# target_ip = "192.168.0.100" # ip on tplink
target_ip = "192.168.0.101" # pi ip on tplink
#target_ip = "127.0.0.1"
target_port = 21000  # Unified port you're using in Flask

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Function to send test data (5 floats: fore, heel, ax, ay, az)

def send_combined_test_data(fore, heel, ax, ay, az):
    i = 0
    while True:
        # Pack data as 5 floats (little-endian by default)
        i += 1
        # data = struct.pack('5f', fore, heel, ax, ay, az)
        data = struct.pack('5f', i, heel, ax, ay, az)
        sock.sendto(data, (target_ip, target_port))
        # print(f"Sent -> Fore: {fore}, Heel: {heel}, ax: {ax}, ay: {ay}, az: {az}", flush=True)
        print(f"Sent -> Fore: {i}, Heel: {heel}, ax: {ax}, ay: {ay}, az: {az}", flush=True)
        time.sleep(0.004) #0.004 (250hz) #app.py (ran locally). cannot do 500hz, can sometimes keep up with 400hz, can do 200hz, i think can keep up with 300hz most of the time

# Example: Fore=900, Heel=850, Simulated accel values
send_combined_test_data(1000.0, 100.0, 2.00, 0.01, 9.81)


print("Done sending", flush=True)

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

# Step 1: Standing still
for _ in range(1000):
    send_packet(900, 900, 0.0, 0.0, 0)

    time.sleep(0.01)

# Step 2: Takeoff 
for i in range(45):
    send_packet(0, 0, 0.0, 0.0, 9.8)  # Jump upward burst
    time.sleep(0.01)

# Step 3: in air (1 seconds)
for _ in range(10):
   send_packet(0, 0, 0.0, 0.0, 0.0)  # Near free-fall
   time.sleep(0.01)

# Step 4: Landing 
for _ in range(10):
    send_packet(900, 900, 0.0, 0.0, 0)  # Downward impact spike
    time.sleep(0.01)

# Step 5: Back to standing
for _ in range(1000):
    send_packet(900, 900, 0.0, 0.0, 0)
    time.sleep(0.01)

print("Complete", flush=True)
'''