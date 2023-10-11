import socket
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 25000

n = 1

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
sock.sendto(n.to_bytes(1, byteorder='big'), (UDP_IP, UDP_PORT))