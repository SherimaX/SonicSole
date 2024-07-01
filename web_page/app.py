from flask import Flask, render_template, request, redirect, url_for, jsonify
import socket
import threading

app = Flask(__name__)

UDP_IP = "127.0.0.1"
UDP_PORT = 25000
bufferSize = 1024
received_data = ""

def read_udp_pressure():
    global received_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        # received_data = data.decode('utf-8')
        received_data = 10
        print("received message: %s" % data)

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

@app.route('/data', methods=['GET'])
def data():
    global received_data
    return jsonify({'data': received_data})

if __name__ == '__main__':
    udp_thread = threading.Thread(target=read_udp_pressure)
    udp_thread.daemon = True
    udp_thread.start()
    app.run(host='0.0.0.0', port=5000)
