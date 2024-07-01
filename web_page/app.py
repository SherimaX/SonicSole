from flask import Flask, render_template, request, redirect, url_for
import socket

app = Flask(__name__)

UDP_IP = "127.0.0.1"
UDP_PORT = 25000
bufferSize = 1024

def read_udp_pressure():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))
    while True: 
        data, addr = sock.recvfrom(1024)
        print("received message: %s" % data)
        return data

def send_udp_data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    n = 1
    sock.sendto(n.to_bytes(1, byteorder='big'), (UDP_IP, UDP_PORT))

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/button', methods=['POST'])
def button():
    send_udp_data()
    return redirect(url_for('index'))

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80)

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
