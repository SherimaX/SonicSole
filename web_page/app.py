from flask import Flask, render_template, request, redirect, url_for, jsonify
import socket
import threading
import time
import csv

app = Flask(__name__)

UDP_IP = "127.0.0.1"
UDP_PORT = 21000
UDP_PORT2 = 20000
bufferSize = 1024
received_heel_data = "0"
received_fore_data = "0"
# heel_list = [0 for _ in range(100)]
totalTime = "0"
recording_time = False
R_heel = 0
G_heel = 255

R_fore = 0
G_fore = 255
greatest_total = 0


submitted_name = "User1"


def update_heel_color(pressure):
    global R_heel, G_heel
    if pressure < 1000:
        G_heel = 255
        R_heel = int((pressure / 1000) * 255)
    elif pressure < 2000:
        R_heel = 255
        G_heel = int(255 - ((pressure - 1000) / 1000) * 255)

def update_fore_color(pressure):
    global R_fore, G_fore
    if pressure < 1000:
        G_fore = 255
        R_fore = int((pressure / 1000) * 255)
    elif pressure < 2000:
        R_fore = 255
        G_fore = int(255 - ((pressure - 1000) / 1000) * 255)

def jumpingScoreInformation():
    global received_fore_data, received_heel_data, submitted_name, greatest_total
    greatest_total = 40
    while True:
            if int(received_heel_data) + int(received_fore_data) > greatest_total:
                print(submitted_name)
                greatest_total = int(received_heel_data) + int(received_fore_data)
                g = open("SonicSoleBalance.txt", "a")
                g.write(submitted_name + "," + str(greatest_total) + "\n")
                g.close()


#For balance.html
i = 1

@app.route('/submit', methods=['POST'])
def submit():
    global submitted_name
    submitted_name = request.form['name']
    return jsonify({"status": "Name submitted successfully"})

def balancing_pressure():
    global totalTime, recording_time, submitted_name, i
    start_time = time.time()
    while True:
        if recording_time and (int(received_heel_data) < 500 and int(received_fore_data) < 500):
            i = 0
            end_time = time.time()
            totalTime = str(end_time - start_time)
            print("Currently Balanced for {} seconds".format(totalTime))
            time.sleep(0.01)
        else:
            recording_time = False
            start_time = time.time()
            print("Total time balanced: {} seconds".format(totalTime))
            # print(submitted_name)
            if i == 0:
                f = open("SonicSole2.txt", "a")
                f.write(submitted_name + ","+ totalTime + "\n")
                f.close()
                i = 1
            time.sleep(0.01)
            

# For index.html

def read_heel_pressure():
    global received_heel_data #, heel_list
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        received_heel_data = int.from_bytes(data, byteorder='little')
        update_heel_color(received_heel_data)
        # time.sleep(0.1)
        

def read_fore_pressure():
    global received_fore_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT2))
    while True:
        data, addr = sock.recvfrom(1024)
        received_fore_data = int.from_bytes(data, byteorder='little')
        update_fore_color(received_fore_data)
        # print("received message: %s" % data)

def send_udp_data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    n = 1
    sock.sendto(n.to_bytes(1, byteorder='big'), (UDP_IP, UDP_PORT))

@app.route('/')
def home():
    return render_template('home.html')

@app.route('/index')
def index():
    return render_template('index.html')

@app.route('/balance')
def balance():
    return render_template('balance.html')

@app.route('/bScoreboard')
def b_scoreboard():
    data = []
    try:
        with open('SonicSole2.txt', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:  # Ensure the row has at least 2 elements
                    data.append({'name': row[0], 'time': float(row[1])})
    except FileNotFoundError:
        return "Error: SonicSole2.txt file not found."
    except ValueError:
        return "Error: Incorrect data format in SonicSole2.txt."
    except Exception as e:
        return f"Error: {e}"

    # Sort data based on total time in descending order
    data.sort(key=lambda x: x['time'], reverse=True)
    
    # Remove the lowest time of any duplicate names
    unique_data = {}
    for entry in data:
        name = entry['name']
        time = entry['time']
        if name not in unique_data:
            unique_data[name] = time
        else:
            if time > unique_data[name]:
                unique_data[name] = time

    # Convert unique_data back to a list of dictionaries
    sorted_data = [{'name': name, 'time': time} for name, time in unique_data.items()]
    sorted_data.sort(key=lambda x: x['time'], reverse=True)
    
    return render_template('bScoreboard.html', data=sorted_data)

@app.route('/jScoreboard')
def j_scoreboard():
    data = []
    try:
        with open('SonicSoleBalance.txt', 'r') as g:
            reader = csv.reader(g)
            for row in reader:
                if len(row) >= 2:  # Ensure the row has at least 2 elements
                    data.append({'name': row[0], 'total': float(row[1])})
    except FileNotFoundError:
        return "Error: SonicSoleBalance.txt file not found."
    except ValueError:
        return "Error: Incorrect data format in SonicSoleBalance.txt."
    except Exception as e:
        return f"Error: {e}"

    # Sort data based on total time in descending order
    data.sort(key=lambda x: x['total'], reverse=True)
    
    # Remove the lowest time of any duplicate names
    unique_data = {}
    for entry in data:
        name = entry['name']
        total = entry['total']
        if name not in unique_data:
            unique_data[name] = total
        else:
            if total > unique_data[name]:
                unique_data[name] = total

@app.route('/assemblyInstructions')
def assembly_instructions():
    return render_template('assemblyInstructions.html')

@app.route('/button', methods=['POST'])
def button():
    send_udp_data()
    return redirect(url_for('index'))

@app.route('/button', methods=['POST'])
def button_pressed():
    global greatest_total
    greatest_total = 40
    return redirect(url_for('index'))


@app.route('/heel_data', methods=['GET'])
def heel_data():
    global received_heel_data
    return jsonify({'data': received_heel_data})

@app.route('/fore_data', methods=['GET'])
def fore_data():
    global received_fore_data
    return jsonify({'data': received_fore_data})

# @app.route('/heel_graph', methods=['GET'])
# def heel_graph():
#     global heel_list
#     return jsonify({'data': heel_list[-100:-1]})

@app.route('/balancing', methods=['GET'])
def balancing():
    global totalTime
    return jsonify({'data': totalTime})

@app.route('/button_click', methods=['POST'])
def button_click():
    global recording_time
    recording_time = True
    return jsonify({"status": "Data transmission started"})

@app.route('/color_data', methods=['GET'])
def color_data():
    global R_heel, G_heel, R_fore, G_fore
    return jsonify({'R_heel': R_heel, 'G_heel': G_heel, 'R_fore': R_fore, 'G_fore': G_fore})


if __name__ == '__main__':
    udp_thread = threading.Thread(target=read_heel_pressure)
    udp_thread2 = threading.Thread(target=read_fore_pressure)
    udp_thread_balance = threading.Thread(target=balancing_pressure)
    udp_thread_jumping = threading.Thread(target=jumpingScoreInformation)
    udp_thread.daemon = True
    udp_thread.start()
    udp_thread2.daemon = True
    udp_thread2.start()
    udp_thread_balance.daemon = True
    udp_thread_balance.start()
    udp_thread_jumping.daemon = True
    udp_thread_jumping.start()

    app.run(host='0.0.0.0', port=5000)

