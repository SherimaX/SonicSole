import socket
import threading
import random


UDP_IP = "127.0.0.1"
bufferSize = 1024
received_heel_data = "HEEL DATA"
received_fore_data = "FORE DATA"

def read_heel_pressure(UDP_PORT):
    global received_heel_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        received_heel_data = int.from_bytes(data, byteorder='little')
        print("received message: %s" % data)