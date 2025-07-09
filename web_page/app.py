from flask import Flask, render_template, request, redirect, url_for, jsonify, send_file
import socket
import threading
import time
import csv
import random

from scipy.integrate import cumulative_trapezoid

import numpy as np

app = Flask(__name__)

#UDP_IP = "127.0.0.1"
UDP_IP = "0.0.0.0"
UDP_PORT = 21000
UDP_PORT2 = 20000

UDP_PORT3 = 22000  # for jump height

bufferSize = 1024
received_heel_data = "0"
received_fore_data = "0"

received_vertical_raw = "0.0" # for jump height
airtime=0

# heel_list = [0 for _ in range(100)]
totalTime = "0"
recording_time = False
R_heel = 0
G_heel = 255

R_fore = 0
G_fore = 255

submitted_name = "User1"
eyes_open = False
greatest_total = 50
first_name = "first"
last_name = "last"

submitted_name2 = "User1"
first_name2 = "first"
last_name2 = "last"

# ForeWalk: global state
forefoot_status = "waiting"
forefoot_time = 0

force_trainer_state = {
    'status': 'idle',           # 'idle', 'calibrating', 'measuring', 'done'
    'max_force': 1,            # to avoid division by zero
    'target_percent': 0,
    'measured_force': 0,
    'error_percent': 0
}

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
    global received_fore_data, received_heel_data, submitted_name2, greatest_total
    curr_submitted_name = submitted_name2
    greatest_total = 50
    while True:
            if(submitted_name2 != curr_submitted_name):
                greatest_total = 50
                curr_submitted_name = submitted_name2
            if int(received_heel_data) + int(received_fore_data) > greatest_total:
                print(submitted_name2)
                greatest_total = int(received_heel_data) + int(received_fore_data)
                g = open("SonicSoleBalance.txt", "a")
                g.write(submitted_name2 + "," + str(greatest_total) + "\n")
                g.close()


#For balance.html
i = 1

@app.route('/submit', methods=['POST'])
def submit():
    global submitted_name, first_name, last_name, eyes_open
    first_name = request.form['first_name']
    last_name = request.form['last_name']
    submitted_name = first_name + " " + last_name
    if "eyes" in request.form:
        eyes_open = request.form['eyes']
        submitted_name += "_" + eyes_open
    else:
        pass

    return jsonify({"status": "Name submitted successfully"})

@app.route('/submit2', methods=['POST'])
def submit2():
    global submitted_name2, first_name2, last_name2
    first_name2 = request.form['first_name2']
    last_name2 = request.form['last_name2']
    submitted_name2 = first_name2 + " " + last_name2
    return jsonify({"status": "Name submitted successfully"})


def balancing_pressure():
    global totalTime, recording_time, submitted_name, i
    start_time = time.time()
    while True:
        if recording_time and (int(received_heel_data) < 500 and int(received_fore_data) < 500):
            i = 0
            end_time = time.time()
            totalTime = "{:.3f}".format(end_time - start_time)
            time.sleep(0.01)
        else:
            recording_time = False
            start_time = time.time()
            if i == 0:
                f = open("SonicSole2.txt", "a")
                f.write(submitted_name + ","+ totalTime + "\n")
                f.close()
                i = 1
            time.sleep(0.01)



# ForeWalk: logic thread for forefoot-only walking
def forefoot_walk_session(threshold=500, duration=10):
    global received_heel_data, received_fore_data, forefoot_status, forefoot_time
    while True:
        if forefoot_status == "start":
            start_time = time.time()
            while time.time() - start_time < duration:
                if int(received_heel_data) > threshold:
                    forefoot_status = "fail"
                    forefoot_time = round(time.time() - start_time, 2)
                    break
                time.sleep(0.01)
            else:
                forefoot_status = "success"
                forefoot_time = round(duration, 2)
        time.sleep(0.01)

# For index.html

# for jump height
def read_vertical_f():
    global received_vertical_raw
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT3))
    while True:
        data, addr = sock.recvfrom(1024)
        try:
            received_vertical_raw = data.decode().strip()
        except:
            continue

    
def estimate_jump_height(accel_data_str, dt=0.001):
    if not accel_data_str:
        return 0.0

    try:
        accel_data = np.array([float(a) for a in accel_data_str])
    except ValueError:
        return 0.0

    # Optional: Remove drift
    from scipy.signal import detrend
    accel_corrected = detrend(accel_data)

    # Integrate to velocity
    velocity = cumulative_trapezoid(accel_corrected, dx=dt, initial=0)

    # Integrate to displacement
    displacement = cumulative_trapezoid(velocity, dx=dt, initial=0)

    # Return maximum height
    return round(np.max(displacement), 4)

   
def get_airtime_and_height(threshold=500):
    global received_heel_data, received_fore_data, received_vertical_raw
    vertical_raw_data = []
    #wait for both heel and forefoot to leave ground
    while True:
        if int(received_heel_data) < threshold and int(received_fore_data) < threshold:
            start_time = time.time()
            break
        time.sleep(0.001)
    
    #record data while in the air
    while True:
        if int(received_heel_data) >= threshold or int(received_fore_data) >= threshold:
            end_time = time.time()
            break
        vertical_raw_data.append(received_vertical_raw)
        time.sleep(0.001)
    
    airtime = end_time - start_time
    jump_height = estimate_jump_height(vertical_raw_data)
    return round(airtime, 4), jump_height

def read_heel_pressure():
    global received_heel_data
    print(f"Read Heel Pressure")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        received_heel_data = int.from_bytes(data, byteorder='little')
        update_heel_color(received_heel_data)
        print(f"Heel Pressure: {received_heel_data}")

def read_fore_pressure():
    global received_fore_data
    print(f"Read Fore Pressure")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT2))
    while True:
        data, addr = sock.recvfrom(1024)
        received_fore_data = int.from_bytes(data, byteorder='little')
        update_fore_color(received_fore_data)
        print(f"Fore Pressure: {received_fore_data}")

# Force Trainer Activity
from threading import Thread
@app.route('/start_force_trainer')
def start_force_trainer():
    global trainer_start_time
    trainer_start_time = time.time()

    # Start the activity in a new background thread
    thread = Thread(target=run_force_trainer)
    thread.start()

    return '', 200

@app.route('/force_trainer_status')
def force_trainer_status():
    global trainer_start_time, force_trainer_state
    if trainer_start_time is None:
        return jsonify(status="idle", time=0.0, target_percent=None, max_force=0)
    elapsed = time.time() - trainer_start_time
    return jsonify(
        status=force_trainer_state['status'],
        time=elapsed,
        target_percent=force_trainer_state.get('target_percent', None),
        max_force=force_trainer_state.get('max_force', 0)
    )



def run_force_trainer():
    global force_trainer_state, received_fore_data

    # Step 1: Calibration
    force_trainer_state['status'] = 'calibrating'
    force_trainer_state['max_force'] = 1
    max_val = 0
    start_time = time.time()
    while time.time() - start_time < 10:
        current_val = int(received_fore_data)
        if current_val > max_val:
            max_val = current_val
        time.sleep(0.01)

    force_trainer_state['max_force'] = max_val if max_val > 0 else 1000 

    # ⚠️ New Step: Cooldown
    force_trainer_state['status'] = 'cooldown'
    time.sleep(3)  # wait without recording anything

    # Step 2: Random target percentage
    force_trainer_state['status'] = 'measuring'
    target_percent = random.choice([10, 20, 30, 40, 50, 60, 70, 80, 90])
    force_trainer_state['target_percent'] = target_percent

    # Step 3: Measure user holding pressure
    readings = []
    start_time = time.time()
    while time.time() - start_time < 12:
        readings.append(int(received_fore_data))
        time.sleep(0.01)

    if not readings:
        avg_force = 0
    else:
        avg_force = sum(readings) / len(readings)

    percent_applied = (avg_force / force_trainer_state['max_force']) * 100
    percent_error = abs(percent_applied - force_trainer_state['target_percent'])

    force_trainer_state['measured_force'] = round(percent_applied, 1)
    force_trainer_state['error_percent'] = round(percent_error, 1)
    force_trainer_state['status'] = 'done'



def send_udp_data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    n = 1
    sock.sendto(n.to_bytes(1, byteorder='big'), (UDP_IP, UDP_PORT))

@app.route('/')
def home():
    return render_template('home.html')

@app.route('/jump')
def jump():
    return render_template('jump.html')

@app.route("/forceSensitivity")
def force_trainer_page():
    return render_template("forceSensitivity.html")


@app.route('/balance')
def balance():
    return render_template('balance.html')

@app.route("/play", methods=["GET", "POST"])
def play():
    return send_file("countdown.mp3")

# ForeWalk: load forefoot walk page
@app.route('/forefoot')
def forefoot():
    return render_template('foreWalk.html')

# ForeWalk: start the timed challenge
@app.route('/start_forefoot', methods=['POST'])
def start_forefoot():
    global forefoot_status, forefoot_time
    forefoot_status = "start"
    forefoot_time = 0
    return jsonify({"status": "started"})

# ForeWalk: status polling endpoint
@app.route('/forefoot_status', methods=['GET'])
def get_forefoot_status():
    return jsonify({'status': forefoot_status, 'time': forefoot_time})

@app.route('/bScoreboard')
def b_scoreboard():
    data = []
    try:
        with open('SonicSole2.txt', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    splitted_name = row[0].split("_")
                    if len(splitted_name) > 1:
                        if splitted_name[1] == "0":
                            data.append({'name': splitted_name[0]+" (Eyes Closed)", 'time':  float(row[1])})
                        else:
                            data.append({'name': splitted_name[0]+" (Eyes Opened)", 'time': float(row[1])})
                    else:
                        data.append({'name': row[0], 'time':  float(row[1])})
    except FileNotFoundError:
        return "Error: SonicSole2.txt file not found."
    except ValueError:
        return "Error: Incorrect data format in SonicSole2.txt."
    except Exception as e:
        return f"Error: {e}"
    data.sort(key=lambda x: x['time'], reverse=True)
    unique_data = {}
    for entry in data:
        name = entry['name']
        time = entry['time']
        if name not in unique_data:
            unique_data[name] = time
        else:
            if time > unique_data[name]:
                unique_data[name] = time
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
                if len(row) >= 2:
                    try:
                        data.append({'name': row[0], 'total': float(row[1])})
                    except ValueError:
                        print(f"Invalid data format in row: {row}")
                        continue
    except FileNotFoundError:
        print("Error: SonicSoleBalance.txt file not found.")
        return "Error: SonicSoleBalance.txt file not found."
    except Exception as e:
        print(f"Unexpected error: {e}")
        return f"Error: {e}"
    unique_data = {}
    for entry in data:
        name = entry['name']
        total = entry['total']
        if name not in unique_data:
            unique_data[name] = total
        else:
            if total > unique_data[name]:
                unique_data[name] = total
    leaderboard_data = [{'name': name, 'total': total} for name, total in unique_data.items()]
    leaderboard_data.sort(key=lambda x: x['total'], reverse=True)
    return render_template('jScoreboard.html', data=leaderboard_data)

@app.route('/assemblyInstructions')
def assembly_instructions():
    return render_template('assemblyInstructions.html')

@app.route('/button', methods=['POST'])
def button():
    send_udp_data()
    return redirect(url_for('jump'))

@app.route('/heel_data', methods=['GET'])
def heel_data():
    global received_heel_data
    return jsonify({'data': received_heel_data})

@app.route('/fore_data', methods=['GET'])
def fore_data():
    global received_fore_data
    return jsonify({'data': received_fore_data})

@app.route('/balancing', methods=['GET'])
def balancing():
    global totalTime
    return jsonify({'data': totalTime})

@app.route('/jump_metrics')
def jump_metrics():
    airtime, height = get_airtime_and_height()
    return jsonify({
        'airtime_seconds': airtime,
        'jump_height_meters': height
    })

@app.route('/force_trainer_results')
def force_trainer_results():
    return jsonify(
        measured_force=force_trainer_state.get('measured_force', 0.0),
        error_percent=force_trainer_state.get('error_percent', 0.0),
        target_percent=force_trainer_state.get('target_percent', 0)
    )


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

    udp_thread_verticalF = threading.Thread(target=read_vertical_f)
    udp_thread_verticalF.daemon = True
    udp_thread_verticalF.start()

    # ForeWalk: start thread
    udp_thread_forefoot = threading.Thread(target=forefoot_walk_session)
    udp_thread_forefoot.daemon = True
    udp_thread_forefoot.start()

    app.run(host='0.0.0.0', port=5000)
