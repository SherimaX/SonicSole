from flask import Flask, render_template, request, redirect, url_for, jsonify
import socket
import threading
import time

app = Flask(__name__)

UDP_IP = "127.0.0.1"
UDP_PORT = 21000
UDP_PORT2 = 20000
bufferSize = 1024
received_heel_data = "0"
received_fore_data = "0"
heel_list = [0 for _ in range(100)]
totalTime = 0

#For balance.html

def balancing_pressure():
    global totalTime, received_heel_data, received_fore_data
    start_time = time.time()
    while (int(received_heel_data) < 90 and int(received_fore_data) < 90):
        end_time = time.time()
        totalTime = end_time - start_time
        print("Currently Balaned for {:.2f} seconds".format(totalTime))
    
    totalTime = end_time - start_time
    print("Total time balanced: {:.2f} seconds".format(totalTime))



# For index.html

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

@app.route('/balance/')
def balance():
    return render_template('balance.html')

@app.route('/balancing', methods=['GET'])
def heel_data():
    global received_heel_data
    return jsonify({'data': received_heel_data})

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

@app.route('/heel_graph', methods=['GET'])
def heel_graph():
    global heel_list
    return jsonify({'data': heel_list[-100:-1]})



if __name__ == '__main__':
    udp_thread = threading.Thread(target=read_heel_pressure)
    udp_thread2 = threading.Thread(target=read_fore_pressure)
    udp_thread_balance = threading.Thread(target=balancing_pressure)
    udp_thread.daemon = True
    udp_thread.start()
    udp_thread2.daemon = True
    udp_thread2.start()
    udp_thread_balance.daemon = True
    udp_thread_balance.start()

    app.run(host='0.0.0.0', port=5000)

