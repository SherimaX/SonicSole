from flask import Flask, render_template, request, redirect, url_for, jsonify
import socket
import threading

app = Flask(__name__)

UDP_IP = "127.0.0.1"
UDP_PORT = 21000
UDP_PORT2 = 20000
bufferSize = 1024
received_heel_data = "HEEL DATA"
received_fore_data = "FORE DATA"
heel_list = [0 for _ in range(100)]


def read_heel_pressure():
    global received_heel_data, heel_list
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        received_heel_data = int.from_bytes(data, byteorder='little')
        heel_list.append(received_heel_data)
        print("received message: %s" % heel_list[-100:-1])
        

def read_fore_pressure():
    global received_fore_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT2))
    while True:
        data, addr = sock.recvfrom(1024)
        received_fore_data = int.from_bytes(data, byteorder='little')
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

@app.route('/heel_data', methods=['GET'])
def heel_data():
    global received_heel_data
    return jsonify({'data': received_heel_data})

@app.route('/fore_data', methods=['GET'])
def fore_data():
    global received_fore_data
    return jsonify({'data': received_fore_data})

if __name__ == '__main__':
    udp_thread = threading.Thread(target=read_heel_pressure)
    udp_thread2 = threading.Thread(target=read_fore_pressure)
    udp_thread.daemon = True
    udp_thread.start()
    udp_thread2.daemon = True
    udp_thread2.start()

    app.run(host='0.0.0.0', port=5000)
