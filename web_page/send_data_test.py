import socket
target_ip = "192.168.50.109"
target_port = 21000 
heel_pressure_value = 800  
data = heel_pressure_value.to_bytes(2)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.sendto(data, (target_ip, target_port))
    print(f"Sent heel pressure: {heel_pressure_value} to {target_ip}:{target_port}")
except Exception as e:
    print(f"Error sending data: {e}")
finally:
    sock.close()
    print("Socket closed.")