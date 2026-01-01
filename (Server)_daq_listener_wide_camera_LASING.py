#!/usr/bin/env python3
import socket
import nidaqmx
from nidaqmx.constants import AcquisitionType, RegenerationMode, WAIT_INFINITELY
import time
import bisect
import threading
import numpy as np
import sys
import os
import ctypes

def reset_network():
    print("[SYSTEM] Performing pre-flight network reset...")
    # Disabling/Enabling requires Admin rights
    os.system('netsh interface set interface "Ethernet" admin=disable')
    os.system('timeout /t 2')
    os.system('netsh interface set interface "Ethernet" admin=enable')
    os.system('arp -d *')
    print("[SYSTEM] Network reset complete.")


# Run this before starting the socket listener
if __name__ == "__main__":
    # Check for Admin rights before attempting reset
    if ctypes.windll.shell32.IsUserAnAdmin():
        reset_network()
    else:
        print("[WARNING] Not running as Admin. Skipping network reset.")


# ---------------------------------------------------------
# CONFIGURATION (Updated for 1920x1440)
# ---------------------------------------------------------
HOST = '127.0.0.1'
PORT = 65432
DEV_NAME = 'Dev1'      # Change this to match your NI MAX device name
X_MAX = 1920           # Updated from 3840
Y_MAX = 1440           # Updated from 2160
AO_RANGE = 6.0         # Output voltage range (+/- 5V)
EXPECTED_CAL_POINTS = 20

# NOTE: Max update rate for USB-6001 is ~150Hz.
DAQ_RATE_HZ = 150.0

# ---------------------------------------------------------
# GLOBAL STATE (Mutable)
# ---------------------------------------------------------
# Initial Calibration Points (Pixel -> Voltage) - Scaled for 1920x1440
X_CALIBRATION_POINTS = [
    (125, -6),    # Scaled from (250, -5.5)
    (310, -4.5),    # Scaled from (620, -4.5)
    (550, -3.5),    # Scaled from (1100, -3.5)
    (733, -2.0),    # Scaled from (1466, -2.0)
    (1008, -1.2),   # Scaled from (2016, -1.2)
    (1243, 0.5),   # Scaled from (2486, -0.5)
    (1500, 3),      # Scaled from (3000, 2)
    (1750, 5)     # Scaled from (3500, 4.5)
]

Y_CALIBRATION_POINTS = [
    (0, -5.5),      # Same
    (200, -4),      # Scaled from (300, -2)
    (400, -2),      # Scaled from (600, -1)
    (600, 0),       # Scaled from (900, 0)
    (800, 2),       # Scaled from (1200, 1)
    (1000, 4),      # Scaled from (1500, 2)
    (1333, 5.5)     # Scaled from (2000, 3.0)
]

# Run-time Lookup Tables
X_LOOKUP_TABLE = []
Y_LOOKUP_TABLE = []

# Calibration Data Loggers
CALIBRATION_VOLTAGE_POINTS = []
CALIBRATION_PIXEL_POINTS_RECEIVED = []

# State Flags
IS_LOGGING_VOLTAGES = False
IS_RECEIVING_PIXELS = False
IS_RECEIVING_BATCH = False
LASER_ON = False  # Added to track lasing state

# DAQ Tasks
SINGLE_POINT_TASK = None
CONTINUOUS_BATCH_TASK = None 
LASER_TASK = None  # Added for digital output task

# Lock for updating lookup tables
lookup_lock = threading.Lock()

# ---------------------------------------------------------
# INTERPOLATION & LOOKUP
# ---------------------------------------------------------
def piecewise_linear_interpolation(coord, calibration_points, ao_range):
    coords = [p[0] for p in calibration_points]
    i = bisect.bisect_left(coords, coord)

    if i == 0:
        return calibration_points[0][1]
    if i == len(calibration_points):
        return calibration_points[-1][1]

    c1, v1 = calibration_points[i - 1]
    c2, v2 = calibration_points[i]

    if c2 == c1:
        return v1

    v_out = v1 + (v2 - v1) * (coord - c1) / (c2 - c1)
    return max(-ao_range, min(ao_range, v_out))


def generate_full_resolution_lookup_table(calibration_points, max_coord, ao_range):
    calibration_points.sort(key=lambda p: p[0])
    lookup = []
    for pixel in range(int(max_coord) + 1):
        voltage = piecewise_linear_interpolation(pixel, calibration_points, ao_range)
        lookup.append(voltage)
    return lookup


def lookup_voltage(pixel_coord, lookup_table, max_coord):
    idx = int(round(pixel_coord))
    idx = max(0, min(int(max_coord), idx))
    return lookup_table[idx]

def update_lookup_tables(x_cal_final, y_cal_final):
    global X_LOOKUP_TABLE, Y_LOOKUP_TABLE
    with lookup_lock:
        X_LOOKUP_TABLE = generate_full_resolution_lookup_table(x_cal_final, X_MAX, AO_RANGE)
        Y_LOOKUP_TABLE = generate_full_resolution_lookup_table(y_cal_final, Y_MAX, AO_RANGE)
        print("ACK: LOOKUP TABLES RECALCULATED based on new calibration data.")

# Initialize lookup tables with the hardcoded initial guess
update_lookup_tables(X_CALIBRATION_POINTS, Y_CALIBRATION_POINTS)


# ---------------------------------------------------------
# DAQ CONTROL FUNCTIONS
# ---------------------------------------------------------
def set_laser_output(state):
    """Sets the digital output for the laser on P0.0."""
    global LASER_TASK
    if LASER_TASK is not None:
        try:
            LASER_TASK.write(state)
        except Exception as e:
            print(f"Error writing to LASER_TASK: {e}")

def create_continuous_batch_task():
    """Creates a new DAQ task configured for continuous, repeating analog output."""
    daq_task = nidaqmx.Task()

    # Add two analog output channels (ao0 for X, ao1 for Y)
    daq_task.ao_channels.add_ao_voltage_chan(f"{DEV_NAME}/ao0", min_val=-AO_RANGE, max_val=AO_RANGE)
    daq_task.ao_channels.add_ao_voltage_chan(f"{DEV_NAME}/ao1", min_val=-AO_RANGE, max_val=AO_RANGE)

    # CRITICAL: Configure timing for continuous output
    daq_task.timing.cfg_samp_clk_timing(
        rate=DAQ_RATE_HZ,
        sample_mode=AcquisitionType.CONTINUOUS,
        samps_per_chan=0
    )
    
    # CRITICAL: Allow buffer regeneration (the cycle)
    daq_task.out_stream.regen_mode = RegenerationMode.ALLOW_REGENERATION

    return daq_task

def stop_continuous_batch():
    global CONTINUOUS_BATCH_TASK, SINGLE_POINT_TASK
    
    # 1. Kill the continuous task FIRST and FORCIBLY
    if CONTINUOUS_BATCH_TASK is not None:
        try:
            # Abort is sometimes safer than stop during an underflow error
            CONTINUOUS_BATCH_TASK.control(nidaqmx.constants.TaskMode.TASK_ABORT) 
            CONTINUOUS_BATCH_TASK.close() 
            print("ACK: CONTINUOUS STREAMING STOPPED AND TASK CLOSED.")
        except Exception as e:
            print(f"Forced close on continuous task: {e}")
        finally:
            CONTINUOUS_BATCH_TASK = None

    # 2. Small delay (0.01s) to let the NI-DAQ driver release the hardware
    time.sleep(0.01)

    # 3. Now that Dev1 is free, use the Single Point task to zero the mirrors
    # try:
    #     if SINGLE_POINT_TASK:
    #         # Re-start the task if it was stopped by a previous error
    #         SINGLE_POINT_TASK.write([0.0, 0.0], auto_start=True)
    # except Exception as e:
    #     print(f"Could not zero mirrors: {e}")


def write_and_start_continuous_batch(x_voltages, y_voltages):
    """Writes the full voltage arrays to the continuous DAQ buffer and starts the task."""
    global CONTINUOUS_BATCH_TASK
    
    # If the task doesn't exist (because it was just closed), create a fresh one
    if CONTINUOUS_BATCH_TASK is None:
        try:
            CONTINUOUS_BATCH_TASK = create_continuous_batch_task()
        except Exception as e:
            print(f"ERROR: Could not recreate continuous batch task: {e}")
            return
            
    num_samples = len(x_voltages)
    if num_samples == 0:
        return

    # Data arranged by channel: [[v_x1, v_x2, ...], [v_y1, v_y2, ...]]
    data_to_write = np.array([x_voltages, y_voltages])
    
    # Ensure the task is stopped before writing a new waveform (safety check)
    try:
        CONTINUOUS_BATCH_TASK.stop()
    except:
        pass 

    try:
        # Define the buffer size and loop length for the new waveform
        CONTINUOUS_BATCH_TASK.timing.cfg_samp_clk_timing(
            rate=DAQ_RATE_HZ,
            sample_mode=AcquisitionType.CONTINUOUS,
            samps_per_chan=num_samples 
        )

        # Write the new data and start the cycle
        CONTINUOUS_BATCH_TASK.write(data_to_write, auto_start=True, timeout=1.0)
        
        print(f"ACK: CONTINUOUS BATCH STARTED. {num_samples} samples cycling at {DAQ_RATE_HZ:.0f} Hz.")

    except Exception as e:
        print(f"ERROR starting continuous batch to DAQ: {e}")
        try:
            CONTINUOUS_BATCH_TASK.close()
        except:
            pass
        CONTINUOUS_BATCH_TASK = None


def handle_batch_data(line):
    """Processes a large, semicolon-separated pixel batch and starts continuous output."""
    try:
        # Parse the string: "x1,y1;x2,y2;x3,y3..."
        point_strings = line.split(";")

        # Convert points to a list of (x_pix, y_pix) tuples
        pixel_points = []
        for s in point_strings:
            x_s, y_s = s.split(",")
            pixel_points.append((int(x_s), int(y_s)))

    except ValueError as e:
        print(f"ERROR parsing batch data: {e}. Data: {line[:50]}...")
        return

    # Convert all pixel points to voltage points using the lookup tables
    x_voltages = []
    y_voltages = []
    with lookup_lock:
        for x_pix, y_pix in pixel_points:
            v_x = lookup_voltage(x_pix, X_LOOKUP_TABLE, X_MAX)
            v_y = lookup_voltage(y_pix, Y_LOOKUP_TABLE, Y_MAX)
            x_voltages.append(v_x)
            y_voltages.append(v_y)

    # Write and Start Continuous Batch
    write_and_start_continuous_batch(x_voltages, y_voltages)


# ---------------------------------------------------------
# MAIN LISTENER LOOP
# ---------------------------------------------------------
def main():
    global IS_LOGGING_VOLTAGES, IS_RECEIVING_PIXELS, IS_RECEIVING_BATCH, LASER_ON
    global SINGLE_POINT_TASK, CONTINUOUS_BATCH_TASK, LASER_TASK

    # Set up DAQ Tasks
    try:
        print(f"Setting up Single-Point DAQ task on {DEV_NAME}...")
        SINGLE_POINT_TASK = nidaqmx.Task()
        SINGLE_POINT_TASK.ao_channels.add_ao_voltage_chan(f"{DEV_NAME}/ao0", min_val=-AO_RANGE, max_val=AO_RANGE)
        SINGLE_POINT_TASK.ao_channels.add_ao_voltage_chan(f"{DEV_NAME}/ao1", min_val=-AO_RANGE, max_val=AO_RANGE)

        # Setup Digital Output Task for Laser Trigger
        print(f"Setting up Digital Output task on {DEV_NAME}/port0/line0...")
        LASER_TASK = nidaqmx.Task()
        LASER_TASK.do_channels.add_do_chan(f"{DEV_NAME}/port0/line0")
        LASER_TASK.start()

    except Exception as e:
        print(f"Error setting up DAQ: {e}")
        sys.exit(1)


    # Socket listener
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen()
            print(f"Listening on {HOST}:{PORT}...")

            conn, addr = s.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"Connected by {addr}")

            # Reset state for initial connection
            CALIBRATION_VOLTAGE_POINTS.clear()
            CALIBRATION_PIXEL_POINTS_RECEIVED.clear()
            IS_LOGGING_VOLTAGES = True
            IS_RECEIVING_PIXELS = False
            IS_RECEIVING_BATCH = False
            LASER_ON = False

            print("--- Ready for Client's DAC stream (Calibration Phase) ---")

            with conn:
                buffer = ""
                while True:
                    data = conn.recv(8192).decode("utf-8")
                    if not data: break
                    buffer += data

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        if line == "CALIBRATION_START":
                            stop_continuous_batch()
                            CALIBRATION_VOLTAGE_POINTS.clear()
                            IS_LOGGING_VOLTAGES = True
                            print("\n[SERVER] RECEIVED CALIBRATION START SIGNAL. Awaiting voltage/pixel pairs.")
                            continue

                        # --- LASING START/STOP HANDLERS ---
                        if line == "LASING_START":
                            LASER_ON = True
                            print("\n[SERVER] LASER ARMED")
                            continue

                        elif line.startswith("DWELL_MS="):
                            try:
                                dwell_time = float(line.split("=")[1]) / 1000.0 # Convert ms to seconds
                                print(f"[SERVER] Dwell set to: {dwell_time*1000:.1f}ms")
                            except:
                                pass
                            continue

                        elif line == "LASING_END":
                            LASER_ON = False
                            set_laser_output(False)
                            print("[SERVER] LASER DISARMED")
                            continue

                        # --- BATCH START/STOP HANDLERS ---
                        if line == "BATCH_START":
                            stop_continuous_batch() 
                            IS_RECEIVING_BATCH = True
                            print("\n[SERVER] RECEIVED BATCH START SIGNAL. Preparing for continuous waveform data...")
                            continue

                        if line == "STOP_STREAMING":
                            stop_continuous_batch()
                            continue

                        if IS_RECEIVING_BATCH:
                            handle_batch_data(line)
                            IS_RECEIVING_BATCH = False
                            continue

                        # --- CALIBRATION HANDLERS (Signals) ---
                        if line == "PIXEL_DUMP_START":
                            stop_continuous_batch() 
                            IS_LOGGING_VOLTAGES = False
                            IS_RECEIVING_PIXELS = True
                            print("\n[SERVER] RECEIVED PIXEL DUMP START SIGNAL. Logging incoming pixels (Phase 2).")
                            CALIBRATION_PIXEL_POINTS_RECEIVED.clear()
                            continue

                        if line == "PIXEL_DUMP_END":
                            IS_RECEIVING_PIXELS = False
                            volt_count = len(CALIBRATION_VOLTAGE_POINTS)
                            pix_count = len(CALIBRATION_PIXEL_POINTS_RECEIVED)

                            if volt_count == pix_count:
                                X_CAL_FINAL = []
                                Y_CAL_FINAL = []
                                for (p_x, p_y), (v_x, v_y) in zip(CALIBRATION_PIXEL_POINTS_RECEIVED, CALIBRATION_VOLTAGE_POINTS):
                                    X_CAL_FINAL.append((p_x, v_x))
                                    Y_CAL_FINAL.append((p_y, v_y))

                                X_CAL_FINAL.sort(key=lambda p: p[0])
                                Y_CAL_FINAL.sort(key=lambda p: p[0])
                                
                                # --- ADDED: PRINT CALIBRATION PAIRS ---
                                print("\n--- FINAL CALIBRATION MAPPINGS ---")
                                print("INDEX | PIXEL (X, Y)     | VOLTAGE (VX, VY)")
                                print("-------------------------------------------")
                                for i in range(len(X_CAL_FINAL)):
                                    # Matching by index since both lists are sorted/aligned
                                    p_x, v_x = X_CAL_FINAL[i]
                                    p_y, v_y = Y_CAL_FINAL[i]
                                    print(f"{i:5} | ({p_x:4}, {p_y:4})   | ({v_x:7.4f}, {v_y:7.4f})")
                                print("-------------------------------------------\n")
                                
                                update_lookup_tables(X_CAL_FINAL, Y_CAL_FINAL)
                            else:
                                print(f"ERROR: Calibration point count mismatch. Logged: {volt_count}V, {pix_count}P. Recalibration aborted.")

                            print("[SERVER] Calibration completed/aborted. Ready for streaming.")
                            continue

                        # --- PIXEL RECEPTION MODE (Phase 2 Data) ---
                        if IS_RECEIVING_PIXELS:
                            try:
                                x_pix, y_pix = map(int, line.split(","))
                                CALIBRATION_PIXEL_POINTS_RECEIVED.append((x_pix, y_pix))
                            except ValueError:
                                print(f"[PIXEL DUMP] Invalid pixel format: {line}")
                            continue

                        # --- SINGLE POINT MODE (Enhanced to handle single or multiple points) ---
                        if "," not in line:
                            print(f"Bad message: {line}")
                            continue
                            
                        stop_continuous_batch() 

                        # Split by semicolon in case the client sent a batch instead of a single point
                        points_to_process = line.split(";")

                        for point in points_to_process:
                            try:
                                x_input, y_input = map(int, point.split(","))
                            except ValueError:
                                print(f"Invalid numbers in point: {point}")
                                continue

                            # Use the CURRENT Lookup Table
                            with lookup_lock:
                                v_x = lookup_voltage(x_input, X_LOOKUP_TABLE, X_MAX)
                                v_y = lookup_voltage(y_input, Y_LOOKUP_TABLE, Y_MAX)

                            # Handle Laser Pulse Logic
                            if LASER_ON:
                                set_laser_output(True)
                                time.sleep(dwell_time)
                                
                            # Write to the DAQ hardware using the single-point task
                            SINGLE_POINT_TASK.write([v_x, v_y], auto_start=True)

                            # Log the voltage if we are in DAC calibration mode (Phase 1)
                            if IS_LOGGING_VOLTAGES:
                                CALIBRATION_VOLTAGE_POINTS.append((v_x, v_y))

                            # print(f"POINT PROCESSED: ({x_input},{y_input}) -> ({v_x:+.3f}V , {v_y:+.3f}V)")


    except Exception as e:
        print(f"Socket error: {e}")

    finally:
        print("Shutting down DAQ tasks...")
        if SINGLE_POINT_TASK:
            try:
                SINGLE_POINT_TASK.write([0.0, 0.0])
            except: pass
            SINGLE_POINT_TASK.close()

        if CONTINUOUS_BATCH_TASK:
            try:
                CONTINUOUS_BATCH_TASK.stop()
                CONTINUOUS_BATCH_TASK.close()
            except: pass

        if LASER_TASK:
            try:
                set_laser_output(False)
                LASER_TASK.close()
            except: pass


if __name__ == "__main__":
    main()
