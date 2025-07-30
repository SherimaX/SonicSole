from tkinter import N
from flask import Flask, render_template, request, redirect, url_for, jsonify, send_file
import socket
import threading
import time
import csv
import random
import struct
import pygame
from scipy.integrate import cumulative_trapezoid
import numpy as np
import logging
from threading import Thread #

# Suppress werkzeug logs
logging.getLogger('werkzeug').disabled = True

app = Flask(__name__)
#UDP_IP = "127.0.0.1"
#UDP_IP = "192.168.0.101" #pi's ip on wifi2
UDP_IP = "0.0.0.0" # accept connections on any available network interface of the server
UDP_PORT = 21000 
bufferSize = 1024
received_heel_data = "0"
received_fore_data = "0"
ax = 0.0 #m/s^2
ay = 0.0
az = 0.0
received_vertical_raw = "0.0" 

airtime=0
jump_metrics_ready = False
last_jump_height = 0.0
last_airtime = 0.0

# heel_list = [0 for _ in range(100)]
totalTime = "0"

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

forefoot_status = "waiting"
forefoot_dist = 0
forefoot_elapsed_time = 0

force_trainer_state = {
    'status': 'idle', # 'idle', 'calibrating', 'measuring', 'done'
    'max_force': 1,            
    'target_percent': 0,
    'measured_force': 0,
    'error_percent': 0
}
vertical_raw_data_UDP = []
combined_data_thread = None
combined_data_running = False

reaction_data = {
    "status": "idle",
    "reaction_time": 0.0
}

threshold = 500
dt = 0.01 #time between samples

pygame.init()
pygame.mixer.init()

@app.route("/play", methods=["GET", "POST"])
def play():
    # play sound using py
    try:
        CountSound = pygame.mixer.Sound("countdown.wav")
        CountSound.set_volume(0.01) 
        CountSound.play()
    except Exception as e:
        print("Error playing sound:", e)
        return jsonify({"error": str(e)}), 500
    return send_file("countdown.wav") #to also play on laptop

# data
def start_combined_data_thread():
    global combined_data_thread, combined_data_running
    if not combined_data_running:
        combined_data_running = True
        combined_data_thread = threading.Thread(target=read_combined_data)
        combined_data_thread.daemon = True
        combined_data_thread.start()

def stop_combined_data_thread():
    global combined_data_running
    print("Stopped")
    combined_data_running = False
    print("combined_data_running in stop thread: {}".format(combined_data_running))

def read_combined_data(): # Combine heel, forefoot, and accelerometer readings on one UDP port
    global received_heel_data, received_fore_data, received_vertical_raw
    global ax, ay, az, vertical_raw_data_UDP, combined_data_running
    print("[UDP Thread] Started reading data")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
    sock.settimeout(0.5)  
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except OSError as e:
        print(f"[UDP Thread] Could not bind socket: {e}")
        combined_data_running = False
        sock.close()
        return
    while combined_data_running:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) < 20:
                continue  
            fore_pressure, heel_pressure, ax_val, ay_val, az_val = struct.unpack('5f', data)
            az_val = round(az_val, 3)
            received_fore_data = int(fore_pressure)
            received_heel_data = int(heel_pressure)
            ax, ay, az = ax_val, ay_val, az_val
            received_vertical_raw = az_val
            vertical_raw_data_UDP.append(az_val)
            update_fore_color(received_fore_data)
            update_heel_color(received_heel_data)
            print("combined_data_running in read thread: {}".format(combined_data_running))
            print("[UDP Thread] Receiving data...")
            print(f"Fore Pressure: {received_fore_data}, Heel Pressure: {received_heel_data}")
        except socket.timeout:
            continue
        except Exception:
            continue
    print("[UDP Thread] Stopped reading data")
    sock.close()

@app.route('/heel_data', methods=['GET'])
def heel_data():
    global received_heel_data
    return jsonify({'data': received_heel_data})

@app.route('/fore_data', methods=['GET'])
def fore_data():
    global received_fore_data
    return jsonify({'data': received_fore_data})

@app.route('/stop_data', methods=['GET', 'POST'])
def stop_data():
    print("[Flask] stop_data called")  
    stop_combined_data_thread()
    return '', 204

# color
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
        G_fore = int(255 - ((pressure - 100) / 1000) * 255)

@app.route('/color_data', methods=['GET'])
def color_data():
    global R_heel, G_heel, R_fore, G_fore
    return jsonify({'R_heel': R_heel, 'G_heel': G_heel, 'R_fore': R_fore, 'G_fore': G_fore})
# jump
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

def estimate_jump_height(accel_data_str):
    global dt;
    print("accel_data_str: {}".format(accel_data_str))
    if not accel_data_str:
        return 0.0
    try:
        accel_data = np.array([float(a) for a in accel_data_str])
    except ValueError:
        return 0.0
    # correct acceleration by removing leading and trailing 0s
    # check if accel_data is all 0s
    non_zero_indices = np.where(accel_data != 0)[0]
    if len(non_zero_indices) == 0:
        pass  # do nothing
    else:   
        accel_data = accel_data[non_zero_indices[0]:non_zero_indices[-1]]
    # accel_corrected = detrend(accel_data)  # Optional: Remove drift. dont reaaly need to because jump is short time
    print("accel_corrected: {}".format(accel_data))
    #accel_data = accel_corrected
    
    velocity = cumulative_trapezoid(accel_data, dx=dt, initial=0)
    print("velocity: {}".format(velocity))
    displacement = cumulative_trapezoid(velocity, dx=dt, initial=0)
    print("displacement: {}".format(displacement))
    
    return round(np.max(displacement), 5)

def get_airtime_and_height():
    global received_heel_data, received_fore_data, received_vertical_raw, vertical_raw_data_UDP
    global threshold, dt
    vertical_raw_data = []
    while True:
        if int(received_heel_data) < threshold and int(received_fore_data) < threshold:
            start_time = time.time()
            print("Takeoff detected")
            break
        time.sleep(0.05)
    while True:
        if int(received_heel_data) >= threshold or int(received_fore_data) >= threshold:
            end_time = time.time()
            print("Landing detected")
            break
        vertical_raw_data.append(received_vertical_raw)
        time.sleep(dt)
    airtime = end_time - start_time
    # print(f"Airtime: {airtime:.4f} seconds")
    #print(f"Samples collected vrdudp: {len(vertical_raw_data_UDP)}")
    #print(f"Samples collected vrd: {len(vertical_raw_data)}")
    if len(vertical_raw_data) < 10:
        print("Not enough samples for jump height. Returning 0.")
        return round(airtime, 5), 0.0
    # jump_height = estimate_jump_height(vertical_raw_data) #double integration approach
    jump_height = ((1/8) * 9.81) * ((airtime) ** 2) #physics approach
    # print(f"Estimated height: {jump_height:.5f} m")
    #vertical_raw_data_UDP = []
    return round(airtime, 4), jump_height

@app.route('/start_jump')
def start_jump():
    print("start_jump clicked")
    global last_airtime, last_jump_height, jump_metrics_ready
    jump_metrics_ready = False
    #start_combined_data_thread()
    airtime, height = get_airtime_and_height()
    stop_combined_data_thread()
    last_airtime = airtime
    last_jump_height = height
    jump_metrics_ready = True
    return jsonify({'status': 'jump measured'})

@app.route('/jump_metrics')
def jump_metrics():
    if not jump_metrics_ready:
        return jsonify({
            'airtime_seconds': 0.0,
            'jump_height_meters': 0.0
        })
    return jsonify({
        'airtime_seconds': last_airtime,
        'jump_height_meters': last_jump_height
    })

# balance
def balancing_pressure():
    global totalTime, submitted_name, recording_time
    global received_heel_data, received_fore_data
    global threshold
    print("[balancing_pressure] Called")
    received_heel_data = "0"
    received_fore_data = "0"
    totalTime = "0"
    start_combined_data_thread()
    print("[balancing_pressure] balance_started")
    start_time = None
    start_delay_period = 0.5 #gives grace period so repeated runs are possible 
    start = time.time()
    while True:
        now = time.time()
        if recording_time:
            if start_time is None:
                start_time = now
            if now - start < start_delay_period:
                time.sleep(0.01)
                continue
            heel = int(received_heel_data)
            fore = int(received_fore_data)

            if heel < threshold and fore < threshold:
                totalTime = "{:.3f}".format(now - start_time)
            else:
                recording_time = False
                print("[balancing_pressure] balance_stopped")
                stop_combined_data_thread()
                break
        else:
            start_time = None
        time.sleep(0.01)
    print("[balancing_pressure] thread complete, combined_data_running:", combined_data_running)

@app.route('/balancing', methods=['GET'])
def balancing():
    global totalTime
    return jsonify({'data': totalTime})

# reaction time
@app.route('/start_reaction')
def start_reaction():
    global reaction_data, threshold, received_fore_data, received_heel_data
    received_fore_data = "0"
    received_heel_data = "0"
    start_combined_data_thread()

    reaction_data = {
        "status": "waiting",
        "reaction_time": 0.0
    }

    delay = random.uniform(3, 6.0)
    print(f"[Reaction] Waiting for {delay:.2f}s")
    start_time = time.time()
    while time.time() - start_time < delay:
        if int(received_heel_data) > threshold or int(received_fore_data) > threshold:
            print("[Reaction] Too early")
            reaction_data["status"] = "invalid"
            stop_combined_data_thread()
            return jsonify({"status": "invalid"})
        time.sleep(0.01)
    beep = pygame.mixer.Sound("beep.wav")
    beep.set_volume(0.025) 
    beep.play()
    print("[Reaction] Beep")
    reaction_data["status"] = "timing"
    reaction_start = time.time()
    while True:
        if int(received_heel_data) > threshold or int(received_fore_data) > threshold:
            reaction_end = time.time()
            reaction_time = round(reaction_end - reaction_start, 4)
            reaction_data["reaction_time"] = reaction_time
            reaction_data["status"] = "success"
            print(f"[Reaction] Success! Reaction time: {reaction_time}")
            stop_combined_data_thread()
            return jsonify({"status": "success"})
        time.sleep(0.001)

@app.route('/reaction_status')
def reaction_status():
    return jsonify(reaction_data)

'''
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
'''
# force sensitivity
@app.route('/start_force_trainer')
def start_force_trainer():
    global trainer_start_time
    trainer_start_time = time.time()
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
    start_combined_data_thread()
    force_trainer_state['status'] = 'calibrating' # Step 1: Calibration
    force_trainer_state['max_force'] = 1
    max_val = 0
    start_time = time.time()
    while time.time() - start_time < 10:
        current_val = int(received_fore_data)
        if current_val > max_val:
            max_val = current_val
        time.sleep(0.01)
    force_trainer_state['max_force'] = max_val if max_val > 0 else 1000 

    force_trainer_state['status'] = 'cooldown' # Step 2: Cooldown
    time.sleep(3)

    force_trainer_state['status'] = 'measuring' # Step 3: Random target percentage
    target_percent = random.choice([10, 20, 30, 40, 50, 60, 70, 80, 90])
    force_trainer_state['target_percent'] = target_percent

    readings = [] # Step 4: Measure pressure hold
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
    stop_combined_data_thread()

@app.route('/force_trainer_results')
def force_trainer_results():
    return jsonify(
        measured_force=force_trainer_state.get('measured_force', 0.0),
        error_percent=force_trainer_state.get('error_percent', 0.0),
        target_percent=force_trainer_state.get('target_percent', 0)
    )

#ForeWalk
@app.route('/start_forefoot', methods=['POST']) 
def start_forefoot():
    global forefoot_status, forefoot_dist, threshold, dt, forefoot_elapsed_time, received_heel_data
    forefoot_status = "running"
    received_heel_data = "0"
    forefoot_dist = 0
    start_combined_data_thread()
    duration = 15.0
    start_time = time.time()
    ax_list = []
   
    while time.time() - start_time < duration:
        heel = int(received_heel_data)
        if heel > threshold:
            forefoot_status = "invalid"
            stop_combined_data_thread()
            forefoot_elapsed_time = round(time.time() - start_time, 1)
            return jsonify({"status": "invalid", "time": forefoot_elapsed_time})
        ax_list.append(ax)
        time.sleep(dt)
    stop_combined_data_thread()
    print(f"Samples collected: {len(ax_list)}")
    print(f"Samples collected: {ax_list}")
    forefoot_dist = round(estimate_distance_from_ax(ax_list), 3)
    forefoot_status = "done"
    return jsonify({"status": "done", "distance_meters": forefoot_dist})

@app.route('/forefoot_status', methods=['GET'])
def get_forefoot_status():
    return jsonify({
        'status': forefoot_status,
        'distance_meters': forefoot_dist,
        'time': forefoot_elapsed_time
    })

def estimate_distance_from_ax(ax_values): #later may want to add decay/kalman for drift
    if len(ax_values) < 2:
        return 0.0
    dt_adjusted = 15.0 / len(ax_values)
    print(f"dt_adjusted: {dt_adjusted}")
    ax_array = np.array(ax_values)
    velocity = cumulative_trapezoid(ax_array, dx=dt_adjusted, initial=0)
    displacement = cumulative_trapezoid(velocity, dx=dt_adjusted, initial=0)
    return float(round(displacement[-1], 3))

# webpage routes
@app.route('/')
def home():
    return render_template('home.html')

@app.route('/jump')
def jump():
    start_combined_data_thread()  
    return render_template('jump.html')

@app.route("/forceSensitivity")
def forceSensitivity():
    return render_template("forceSensitivity.html")

@app.route('/balance')
def balance():
    global totalTime, recording_time
    totalTime = "0"
    recording_time = True
    return render_template('balance.html')

@app.route('/assemblyInstructions')
def assembly_instructions():
    start_combined_data_thread()
    return render_template('assemblyInstructions.html')

@app.route('/reaction')
def reaction():
    return render_template('reaction.html')

@app.route('/foreWalk')
def foreWalk():
    return render_template('foreWalk.html')

# scoreboard
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
        # return f"Error: {e}"
        return "Error"
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
                        # print(f"Invalid data format in row: {row}")
                        continue
    except FileNotFoundError:
        print("Error: SonicSoleBalance.txt file not found.")
        return "Error: SonicSoleBalance.txt file not found."
    except Exception as e:
        # print(f"Unexpected error: {e}")
        # return f"Error: {e}"
        return "Error"
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
recording_time = True

# misc
def send_udp_data(): #unsure what this is for
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    n = 1
    sock.sendto(n.to_bytes(1, byteorder='big'), (UDP_IP, UDP_PORT))

@app.route('/button', methods=['POST'])
def button():
    send_udp_data()
    return redirect(url_for('jump'))

@app.route('/button_click', methods=['POST'])
def button_click():
    global recording_time, totalTime
    recording_time = True
    totalTime = "0"
    thread = threading.Thread(target=balancing_pressure)
    thread.daemon = True
    thread.start()
    return jsonify({"status": "Data transmission started"})

# main
if __name__ == '__main__':
    udp_thread_jumping = threading.Thread(target=jumpingScoreInformation)
    udp_thread_jumping.daemon = True
    udp_thread_jumping.start()
    #udp_thread_combined = threading.Thread(target=read_combined_data)
    #udp_thread_combined.daemon = True
    #udp_thread_combined.start()
    # ForeWalk: start thread
    # ForeWalk: start thread
    # udp_thread_forefoot = threading.Thread(target=forefoot_walk_session)
    # udp_thread_forefoot.daemon = True
    # udp_thread_forefoot.start()
    app.run(host='0.0.0.0', port=5000, debug=False)