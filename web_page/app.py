from tkinter import N
from flask import Flask, render_template, request, redirect, url_for, jsonify, send_file
import socket
import threading
import time    
import csv
import random
import struct
import pygame
import numpy as np
import logging
from threading import Thread 

import queue 

logging.getLogger('werkzeug').disabled = True #Suppress werkzeug logs

app = Flask(__name__)
UDP_IP = "127.0.0.1" #accept data from localhost
#UDP_IP = "0.0.0.0" # accept connections on any available network interface of the server
UDP_PORT = 21000 
bufferSize = 1024

udp_queue = queue.Queue(maxsize=1) #hold latest packet only

received_heel_data = "0"
received_fore_data = "0"
ax = 0.0 #m/s^2
ay = 0.0
az = 0.0
g = 9.81 #m/s^2
received_vertical_raw = "0.0" 

airtime=0
jump_metrics_ready = False
last_jump_height = 0.0
last_airtime = 0.0

jump_vertical_buffer = []
jump_collecting = False
jump_lock = threading.Lock()

# heel_list = [0 for _ in range(100)]
totalTime = "0"

R_heel = 0
G_heel = 255
R_fore = 0
G_fore = 255

submitted_name = "User1"
eyes_open = False
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
percent_error = 0

vertical_raw_data_UDP = []
combined_data_thread = None
combined_data_running = False

reaction_data = {
    "status": "idle",
    "reaction_time": 0.0
}
reaction_time = "0"

threshold_fore = 350
threshold_heel = 350
dt = 0.009 #time (s) between samples from udp (its not really 100hz its closer to about 0.009s between samples (111hz))

pygame.init()
pygame.mixer.init()
CountSound = pygame.mixer.Sound("countdown.wav")
CountSound.set_volume(0.0) 

# cumulative trapezoid without scipy because the pi cannot download it
def cumulative_trapezoid_manual(y, dx=1.0, initial=0):
    y = np.asarray(y, dtype=np.float64)
    n = y.shape[0]

    if n < 2:
        return np.array([initial]) if initial is not None else np.array([])

    # Compute trapezoid integration (without initial)
    result = np.empty(n - 1, dtype=np.float64)
    for i in range(n - 1):
        result[i] = 0.5 * (y[i] + y[i + 1]) * dx
    cumulative = np.cumsum(result)

    if initial is not None:
        return np.insert(cumulative, 0, initial)
    else:
        return cumulative

'''
# piano activity attempt 1 (in progress)
distance_from_origin = 0.0
az_history = []
last_step_time = None
origin_set = False
note_thresholds = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4]  # meters
note_sounds = []
note_files = ["a.wav", "b.wav", "c.wav", "d.wav", "e.wav", "f.wav", "g.wav"]
note_sounds = [pygame.mixer.Sound(f) for f in note_files]

def handle_horizontal_movement():
    global az_history, dt, origin_set, distance_from_origin, last_step_time, threshold_heel, threshold_fore

    if not origin_set:
        if int(received_heel_data) > threshold_heel or int(received_fore_data) > threshold_fore:
            print("[Piano] Origin step detected.")
            az_history = []
            origin_set = True
            last_step_time = time.time()
        return

    if int(received_heel_data) < threshold_heel and int(received_fore_data) < threshold_fore:
        az_history.append(az)

    if origin_set and (int(received_heel_data) > threshold_heel or int(received_fore_data) > threshold_fore):
        print("[Piano] Foot down again, estimating distance.")
        if len(az_history) < 2:
            return

        dt_adjusted = (time.time() - last_step_time) / len(az_history)
        velocity = cumulative_trapezoid_manual(az_history, dx=dt_adjusted, initial=0)
        displacement = cumulative_trapezoid_manual(velocity, dx=dt_adjusted, initial=0)

        distance_from_origin = abs(displacement[-1])
        print(f"[Piano] Horizontal distance: {distance_from_origin:.3f} m")

        play_note_based_on_distance(distance_from_origin)

        az_history = []
        last_step_time = time.time()

def play_note_based_on_distance(distance):
    for i, threshold in enumerate(note_thresholds):
        if distance < threshold:
            note_sounds[i].play()
            print(f"[Piano] Played note {i} for distance {distance:.2f} m")
            time.sleep(0.2)
            return
    note_sounds[-1].play()
    print(f"[Piano] Played highest note for distance {distance:.2f} m")
    time.sleep(0.2)

@app.route('/piano_start')
def piano_activity():
    print("[Piano] Starting activity...")
    
    # Start the combined data thread if not already running
    start_combined_data_thread()
    
    # Reset any piano-specific variables
    global distance_from_origin, az_history, origin_set
    distance_from_origin = 0.0
    az_history = []
    origin_set = False
    
    # Start piano loop in a thread
    threading.Thread(target=piano_loop, daemon=True).start()
    
    return jsonify({"status": "Piano activity started"})

def piano_loop():
    global distance_from_origin
    while True:
        # Only run if origin is set
        if origin_set:
            handle_horizontal_movement()
        time.sleep(0.01)  

def stop_piano_activity():
    print("[Piano] Stopping activity...")
    stop_combined_data_thread()  
    global origin_set, az_history, distance_from_origin
    origin_set = False
    az_history = []
    distance_from_origin = 0.0
    print("[Piano] Activity stopped")
'''
# sound
@app.route("/play", methods=["GET", "POST"])
def play():
    # play sound using py
    try:
        CountSound.play()
    except Exception as e:
        print("Error playing sound:", e)
        return jsonify({"error": str(e)}), 500
    return send_file("countdown.wav") #to also play on laptop

#data
def start_combined_data_thread():
    global combined_data_thread, combined_data_running
    if not combined_data_running:
        combined_data_running = True
        combined_data_thread = threading.Thread(target=read_combined_data)
        combined_data_thread.daemon = True
        combined_data_thread.start()

        threading.Thread(target=process_udp_data, daemon=True).start()

def stop_combined_data_thread():
    global combined_data_running
    print("Stopped")
    combined_data_running = False
    print("combined_data_running in stop thread: {}".format(combined_data_running))

def read_combined_data(): #this function to read data and other function to "process" data
    global received_heel_data, received_fore_data, received_vertical_raw, ax, ay, az, g, combined_data_running
    
    print("[UDP Thread] Started reading data")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)
    sock.settimeout(0.1)
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
            # fast updates only
            received_fore_data = int(fore_pressure)
            received_heel_data = int(heel_pressure)
            ax, ay, az = ax_val * g, ay_val * g, az_val * g
            received_vertical_raw = ay_val
            # push to queue (drop old if full)
            if not udp_queue.full():
                udp_queue.put_nowait((received_fore_data, received_heel_data, ax, ay, az))
        except (socket.timeout, BlockingIOError):
            continue
        except Exception as e:
            print(f"[UDP Thread] Error: {e}")
            continue
    sock.close()
    print("[UDP Thread] Stopped reading data")

def process_udp_data(): #this function to do the slow work and the other to read as attampt to improve how fast can read data
    global received_heel_data, received_fore_data, ax, ay, az, jump_vertical_buffer, jump_collecting
    while combined_data_running:
        try:
            fore, heel, ax_val, ay_val, az_val = udp_queue.get(timeout=0.1)
            # received_fore_data = fore
            # received_heel_data = heel
            # ax, ay, az = ax_val, ay_val, az_val
            if jump_collecting:  # If jump recording is active, collect ay_val
                with jump_lock:
                    jump_vertical_buffer.append(ay_val)
            update_fore_color(fore)
            update_heel_color(heel)
            #print(f"Fore:{fore}, Heel:{heel}, ax:{ax_val:.2f}, ay:{ay_val:.2f}, az:{az_val:.2f}")
            print(f"Fore:{fore}, Heel:{heel}, ax:{ax:.2f}, ay:{ay:.2f}, az:{az:.2f}")
        except queue.Empty:
            continue

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
    if pressure < 1500:
        G_heel = 255
        R_heel = int((pressure / 1000) * 255)
    elif pressure < 3000:
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
    
    velocity = cumulative_trapezoid_manual(accel_data, dx=dt, initial=0)
    print("velocity: {}".format(velocity))
    dist = cumulative_trapezoid_manual(velocity, dx=dt, initial=0)
    print("displacement: {}".format(dist))

    return round(np.max(dist), 5)
'''
def get_airtime_and_height():
    global received_heel_data, received_fore_data, received_vertical_raw, vertical_raw_data_UDP, threshold_heel, threshold_fore, dt
    vertical_raw_data = []
    while True:
        if int(received_heel_data) < threshold_heel and int(received_fore_data) < threshold_fore:
            start_time = time.time()
            print("Takeoff detected")
            break
        time.sleep(0.05)
    while True:
        if int(received_heel_data) >= threshold_heel or int(received_fore_data) >= threshold_fore:
            end_time = time.time()
            stop_combined_data_thread()
            print("Landing detected")
            break
        vertical_raw_data.append(received_vertical_raw)
        time.sleep(dt) #soem time.sleep is needed but this way is not ideal
    airtime = end_time - start_time
    # print(f"Airtime: {airtime:.4f} seconds")
    #print(f"Samples collected vrdudp: {len(vertical_raw_data_UDP)}")
    #print(f"Samples collected vrd: {len(vertical_raw_data)}")
    if len(vertical_raw_data) < 10:
        print("Not enough samples for jump height. Returning 0.")
        return round(airtime, 5), 0.0
    jump_height = estimate_jump_height(vertical_raw_data) #double integration of acceleration approach (IMU)
    #jump_height = ((1/8) * 9.81) * ((airtime) ** 2) #physics formula approach. I htink this works pretty well.
    # print(f"Estimated height: {jump_height:.5f} m")
    #vertical_raw_data_UDP = []
    return round(airtime, 4), jump_height'''

def get_airtime_and_height():
    global received_heel_data, received_fore_data, jump_vertical_buffer, jump_collecting
    while True:
        if int(received_heel_data) < threshold_heel and int(received_fore_data) < threshold_fore:
            start_time = time.time()
            jump_vertical_buffer = []  # Reset buffer
            jump_collecting = True
            print("Takeoff detected")
            break
        #time.sleep(0.01)
    while True:
        if int(received_heel_data) >= threshold_heel or int(received_fore_data) >= threshold_fore:
            end_time = time.time()
            jump_collecting = False
            stop_combined_data_thread()
            print("Landing detected")
            break
        time.sleep(0.01)
    airtime = end_time - start_time
    samples = []
    # Safely copy collected data
    with jump_lock:
        samples = jump_vertical_buffer[:]
    if len(samples) < 10:
        print("Not enough samples for jump height. Returning 0.")
        return round(airtime, 5), 0.0
    jump_height = estimate_jump_height(samples) #double acceleration integration approach
    #jump_height = ((1/8) * 9.81) * ((airtime) ** 2) #physics formula approach. I think this works pretty well
    #print(f"Estimated height: {jump_height:.5f} m")
    return round(airtime, 4), jump_height

@app.route('/start_jump')
def start_jump():
    print("start_jump clicked")
    global last_airtime, last_jump_height, jump_metrics_ready, submitted_name2
    jump_metrics_ready = False
    #start_combined_data_thread()
    airtime, height = get_airtime_and_height()
    #stop_combined_data_thread()
    last_airtime = airtime
    last_jump_height = height
    jump_metrics_ready = True

    with open("SonicSoleJump.txt", "a") as f:
        f.write(f"{submitted_name2},{last_jump_height}\n")

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
    global totalTime, submitted_name, recording_time, received_heel_data, received_fore_data, threshold_heel, threshold_fore
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

            if heel < threshold_heel and fore < threshold_fore:
                totalTime = "{:.3f}".format(now - start_time)
            else:
                recording_time = False
                print("[balancing_pressure] balance_stopped")
                    
                with open("SonicSoleBalance.txt", "a") as f:
                    f.write(f"{submitted_name},{totalTime}\n")

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

@app.route('/button_click', methods=['POST'])
def button_click():
    global recording_time, totalTime, CountSound
    CountSound.play()
    time.sleep(3.12) #sleeps so countdown can run before timer begins
    recording_time = True
    totalTime = "0"
    thread = threading.Thread(target=balancing_pressure)
    thread.daemon = True
    thread.start()
    return jsonify({"status": "Data transmission started"})

# reaction time
@app.route('/start_reaction')
def start_reaction():
    global reaction_data, threshold_heel, threshold_fore, received_fore_data, received_heel_data, submitted_name2, reaction_time
    received_fore_data = "0"
    received_heel_data = "0"
    reaction_time = "0"
    start_combined_data_thread()
    reaction_data = {
        "status": "waiting",
        "reaction_time": 0.0
    }
    delay = random.uniform(3, 6.0)
    print(f"[Reaction] Waiting for {delay:.2f}s")
    start_time = time.time()
    while time.time() - start_time < delay:
        if int(received_heel_data) > threshold_heel or int(received_fore_data) > threshold_fore:
            print("[Reaction] Too early")
            reaction_data["status"] = "invalid"
            stop_combined_data_thread()
            return jsonify({"status": "invalid"})
        time.sleep(0.005)
    beep = pygame.mixer.Sound("beep.wav")
    beep.set_volume(0.0) 
    beep.play()
    print("[Reaction] Beep")
    reaction_data["status"] = "timing"
    reaction_start = time.time()
    while True:
        if int(received_heel_data) > threshold_heel or int(received_fore_data) > threshold_fore:
            reaction_end = time.time()
            reaction_time = round(reaction_end - reaction_start, 4)
            reaction_data["reaction_time"] = reaction_time
            reaction_data["status"] = "success"
            print(f"[Reaction] Success! Reaction time: {reaction_time}")
          
            with open("SonicSoleReaction.txt", "a") as f:
                f.write(f"{submitted_name2},{reaction_time}\n")
            
            stop_combined_data_thread()
            return jsonify({"status": "success"})
        time.sleep(0.001)

@app.route('/reaction_status')
def reaction_status():
    return jsonify(reaction_data)

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
    global force_trainer_state, received_fore_data, submitted_name2, percent_error
    start_combined_data_thread()
    force_trainer_state['status'] = 'calibrating' # Step 1: Calibration
    force_trainer_state['max_force'] = 1
    max_val = 0
    percent_error = 0
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
    target_percent = random.choice([20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105])
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
    time.sleep(1)
    percent_error = round(percent_error, 3)

    with open("SonicSoleForceSense.txt", "a") as f:
        f.write(f"{submitted_name2},{percent_error}\n")

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
    global forefoot_status, forefoot_dist, threshold_heel, dt, forefoot_elapsed_time, received_heel_data, submitted_name2
    forefoot_status = "running"
    received_heel_data = "0"
    forefoot_dist = 0
    start_combined_data_thread()
    duration = 15.0
    start_time = time.time()
    ax_list = []

    #for vector norm (fvn)
    ay_list = []
    az_list = []

    while time.time() - start_time < duration:
        heel = int(received_heel_data)
        if heel > threshold_heel:
            forefoot_status = "invalid"
            stop_combined_data_thread()
            forefoot_elapsed_time = round(time.time() - start_time, 1)
            return jsonify({"status": "invalid", "time": forefoot_elapsed_time})
        ax_list.append(ax)

        ay_list.append(ay) #fvn
        az_list.append(az) #fvn

      #  ax_list.append(az)
        time.sleep(dt)
    stop_combined_data_thread()
    print(f"Samples collected: {len(ax_list)}")
    print(f"Samples collected: {ax_list}")

    # print(f"Samples collected: {len(ay_list)}") #fvn
    # print(f"Samples collected: {ay_list}") #fvn
    # print(f"Samples collected: {len(az_list)}") #fvn
    # print(f"Samples collected: {az_list}") #fvn

    forefoot_dist = round(estimate_distance_from_ax(ax_list,), 3)

    # forefoot_dist = round(estimate_distance_from_ax_vector_norm(ax_list, ay_list, az_list), 3) #fvn

    forefoot_status = "done"
                        
    with open("SonicSoleWalk.txt", "a") as f:
        f.write(f"{submitted_name2},{forefoot_dist}\n")

    return jsonify({"status": "done", "distance_meters": forefoot_dist})

@app.route('/forefoot_status', methods=['GET'])
def get_forefoot_status():
    return jsonify({
        'status': forefoot_status,
        'distance_meters': forefoot_dist,
        'time': forefoot_elapsed_time
    })
'''
def estimate_distance_from_ax(ax_values): #later may want to add decay/kalman/low pass filter for drift
    if len(ax_values) < 2:
        return 0.0
    dt_adjusted = 15.0 / len(ax_values) #adjust dt based on num of packets actually received
    print(f"dt_adjusted: {dt_adjusted}")
    ax_array = np.array(ax_values)
    displacement = 0
    velocity = cumulative_trapezoid_manual(ax_array, dx=dt_adjusted, initial=0)
    speed = np.abs(velocity) 
    displacement = cumulative_trapezoid_manual(velocity, dx=dt_adjusted, initial=0)
   # displacement = cumulative_trapezoid_manual(speed, dx=dt_adjusted, initial=0) #distance not displacement

    return float(round(displacement[-1], 3))

'''

def estimate_distance_from_ax(ax_values):
    if len(ax_values) < 2:
        return 0.0
    dt_adjusted = 15.0 / len(ax_values) #adjust dt based on num of packets actually received
    ax_array = np.array(ax_values, dtype=np.float64)

    #  Bias Calibration (remove constant offset). This assumes user is stationary for first ~100 samples. maybe introduce calibration period.

    # bias = np.mean(ax_array[:100])  # estimate bias from first N samples
    # ax_array = ax_array - bias

    #  Low-Pass Filter (smooth out high-frequency noise)

    # alpha = 0.1  # 0 < alpha < 1, smaller = smoother
    # ax_filtered = np.zeros_like(ax_array)
    # ax_filtered[0] = ax_array[0]
    # for i in range(1, len(ax_array)):
    #     ax_filtered[i] = alpha * ax_array[i] + (1 - alpha) * ax_filtered[i - 1]
    # ax_array = ax_filtered

    #  High-Pass Filter (remove drift/very low frequency bias)

    # hp_alpha = 0.9  # closer to 1 keeps high freq, removes drift
    # ax_hp = np.zeros_like(ax_array)
    # ax_hp[0] = ax_array[0]
    # for i in range(1, len(ax_array)):
    #     ax_hp[i] = hp_alpha * (ax_hp[i-1] + ax_array[i] - ax_array[i-1])
    # ax_array = ax_hp

    #  Integration
    velocity = cumulative_trapezoid_manual(ax_array, dx=dt_adjusted, initial=0)
    #speed = np.abs(velocity) 
    #path_distance = cumulative_trapezoid_manual(speed, dx=dt_adjusted, initial=0)
    #return float(round(path_distance[-1], 3))
    displacement = cumulative_trapezoid_manual(velocity, dx=dt_adjusted, initial=0)
    return float(round(displacement[-1], 3))

def estimate_distance_from_ax_vector_norm(ax_values, ay_values, az_values): #fvn
    if len(ax_values) < 2:
        return 0.0
    dt_adjusted = 15.0 / len(ax_values) #adjust dt based on num of packets actually received
    ax_array = np.array(ax_values, dtype=np.float64)
    ay_array = np.array(ay_values, dtype=np.float64)
    az_array = np.array(az_values, dtype=np.float64)

    #  Bias Calibration (remove constant offset). This assumes user is stationary for first ~100 samples. maybe introduce calibration period.

    # bias = np.mean(ax_array[:100])  # estimate bias from first N samples
    # ax_array = ax_array - bias

    #  Low-Pass Filter (smooth out high-frequency noise)
    # this filter was made for just ax, not vector norm.
    # alpha = 0.1  # 0 < alpha < 1, smaller = smoother
    # ax_filtered = np.zeros_like(ax_array)
    # ax_filtered[0] = ax_array[0]
    # for i in range(1, len(ax_array)):
    #     ax_filtered[i] = alpha * ax_array[i] + (1 - alpha) * ax_filtered[i - 1]
    # ax_array = ax_filtered

    #  High-Pass Filter (remove drift/very low frequency bias)
    # this filter was made for just ax, not vector norm.
    # hp_alpha = 0.9  # closer to 1 keeps high freq, removes drift
    # ax_hp = np.zeros_like(ax_array)
    # ax_hp[0] = ax_array[0]
    # for i in range(1, len(ax_array)):
    #     ax_hp[i] = hp_alpha * (ax_hp[i-1] + ax_array[i] - ax_array[i-1])
    # ax_array = ax_hp

    #  Integration
    velocity_x = cumulative_trapezoid_manual(ax_array, dx=dt_adjusted, initial=0)
    velocity_y = cumulative_trapezoid_manual(ay_array, dx=dt_adjusted, initial=0)
    velocity_z = cumulative_trapezoid_manual(az_array, dx=dt_adjusted, initial=0)
    # velocity = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2) #vector norm for velocity
    # displacement = cumulative_trapezoid_manual(velocity, dx=dt_adjusted, initial=0)
    # return float(round(displacement[-1], 3))
    displacement_x = cumulative_trapezoid_manual(velocity_x, dx=dt_adjusted, initial=0)
    displacement_y = cumulative_trapezoid_manual(velocity_y, dx=dt_adjusted, initial=0)
    displacement_z = cumulative_trapezoid_manual(velocity_z, dx=dt_adjusted, initial=0)
    displacement_vector_norm = np.sqrt(displacement_x**2 + displacement_z**2) #vector norm for displacement
    return float(round(displacement_vector_norm[-1], 3))
    #speed_x = np.abs(velocity_x) 
    #speed_y = np.abs(velocity_y) 
    #speed_z = np.abs(velocity_z) 
    #path_distance_X = cumulative_trapezoid_manual(speed_x, dx=dt_adjusted, initial=0)
    #path_distance_Y = cumulative_trapezoid_manual(speed_y, dx=dt_adjusted, initial=0)
    #path_distance_Z = cumulative_trapezoid_manual(speed_z, dx=dt_adjusted, initial=0)
    #path_distance_vector_norm = np.sqrt(path_distance_X**2 + path_distance_Y**2 + path_distance_Z**2) #vector norm for path distance
    #return float(round(path_distance_vector_norm[-1], 3))
    

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

@app.route('/piano')
def piano():
    return render_template('piano.html')

#plot routes


# Start the combined data thread (safe to call multiple times)
@app.route('/start_stream', methods=['GET'])
def start_stream():
    start_combined_data_thread()
    return jsonify({"status": "stream started"}), 200

# Stop the combined data thread (reuse your stop endpoint if you like)
@app.route('/stop_stream', methods=['GET'])
def stop_stream():
    stop_combined_data_thread()
    return jsonify({"status": "stream stopped"}), 200

# Return the most-recent accelerometer values
@app.route('/accel', methods=['GET'])
def accel_values():
    # ax, ay, az are global floats updated by your UDP thread
    return jsonify({
        'ax': round(ax, 4),
        'ay': round(ay, 4),
        'az': round(az, 4),
        'ts': time.time()
    })

# Render the live plotting page
@app.route('/accel_view')
def accel_view():
    start_combined_data_thread()
    return render_template('accel.html')


# scoreboards
@app.route('/bScoreboard')
def b_scoreboard():
    data = []
    try:
        with open('SonicSoleBalance.txt', 'r') as f:
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
        return "Error: SonicSoleBalance.txt file not found."
    except ValueError:
        return "Error: Incorrect data format in SonicSoleBalance.txt."
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
        with open('SonicSoleJump.txt', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    try:
                        data.append({'name': row[0], 'last_jump_height': float(row[1])})
                    except ValueError:
                        continue
    except FileNotFoundError:
        print("Error: SonicSoleJump.txt file not found.")
        return "Error: SonicSoleJump.txt file not found."
    except Exception as e:
        return "Error"
    unique_data = {}
    for entry in data:
        name = entry['name']
        total = entry['last_jump_height']
        if name not in unique_data or total > unique_data[name]:
            unique_data[name] = total
    leaderboard_data = [{'name': name, 'last_jump_height': total} for name, total in unique_data.items()]
    leaderboard_data.sort(key=lambda x: x['last_jump_height'], reverse=True)
    return render_template('jScoreboard.html', data=leaderboard_data)

@app.route('/rScoreboard')
def r_scoreboard():
    data = []
    try:
        with open('SonicSoleReaction.txt', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    try:
                        data.append({'name': row[0], 'reaction_time': float(row[1])})
                    except ValueError:
                        continue
    except FileNotFoundError:
        return "Error: SonicSoleReaction.txt file not found."
    except Exception:
        return "Error"
    
    # Keep best (lowest) time per user
    unique_data = {}
    for entry in data:
        name = entry['name']
        time = entry['reaction_time']
        if name not in unique_data or time < unique_data[name]:
            unique_data[name] = time
    
    leaderboard_data = [{'name': name, 'reaction_time': time} for name, time in unique_data.items()]
    leaderboard_data.sort(key=lambda x: x['reaction_time'])  # Ascending (lower is better)

    return render_template('rScoreboard.html', data=leaderboard_data)

@app.route('/fScoreboard')
def f_scoreboard():
    data = []
    try:
        with open('SonicSoleForceSense.txt', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    try:
                        data.append({'name': row[0], 'percent_error': float(row[1])})
                    except ValueError:
                        continue
    except FileNotFoundError:
        return "Error: SonicSoleForceSense.txt file not found."
    except Exception:
        return "Error"
    
    # Keep best (lowest) percent error per user
    unique_data = {}
    for entry in data:
        name = entry['name']
        error = entry['percent_error']
        if name not in unique_data or error < unique_data[name]:
            unique_data[name] = error
    
    leaderboard_data = [{'name': name, 'percent_error': error} for name, error in unique_data.items()]
    leaderboard_data.sort(key=lambda x: x['percent_error'])  # Ascending (lower is better)

    return render_template('fScoreboard.html', data=leaderboard_data)

@app.route('/wScoreboard')
def w_scoreboard():
    data = []
    try:
        with open('SonicSoleWalk.txt', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    try:
                        data.append({'name': row[0], 'forefoot_dist': float(row[1])})
                    except ValueError:
                        continue
    except FileNotFoundError:
        return "Error: SonicSoleWalk.txt file not found."
    except Exception:
        return "Error"
    
    # Keep best (highest) distance per user
    unique_data = {}
    for entry in data:
        name = entry['name']
        dist = entry['forefoot_dist']
        if name not in unique_data or dist > unique_data[name]:
            unique_data[name] = dist
    
    leaderboard_data = [{'name': name, 'forefoot_dist': dist} for name, dist in unique_data.items()]
    leaderboard_data.sort(key=lambda x: x['forefoot_dist'], reverse=True)  

    return render_template('wScoreboard.html', data=leaderboard_data)

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

# misc (not sure what these are for)
def send_udp_data(): 
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    n = 1
    sock.sendto(n.to_bytes(1, byteorder='big'), (UDP_IP, UDP_PORT))

@app.route('/button', methods=['POST'])
def button():
    send_udp_data()
    return redirect(url_for('jump'))

# main
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)