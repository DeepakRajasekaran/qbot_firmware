#!/usr/bin/env python3
import serial
import threading
import time
import sys
import datetime

# --- CONFIGURATION ---
DEFAULT_PORT = '/dev/ttyUSB0'  # Change to /dev/ttyACM0 if using an UNO/Mega
BAUD_RATE = 115200
# ---------------------

running = True

def receive_data(ser, log_file):
    """Thread function to continuously read from serial and log to file."""
    global running
    while running:
        try:
            if ser.in_waiting > 0:
                raw_line = ser.readline()
                try:
                    line = raw_line.decode('utf-8').strip()
                    if line:
                        timestamp = datetime.datetime.now()
                        # Decode telemetry if it matches the CSV format (f;...)
                        if line.startswith("f;"):
                            try:
                                # Format: f;poseX;poseY;poseTheta;inputL;inputR
                                parts = line.split(';')
                                if len(parts) >= 6:
                                    px, py, pth = float(parts[1]), float(parts[2]), float(parts[3])
                                    rl, rr = float(parts[4]), float(parts[5])
                                    
                                    decoded = f"Pose({px:.2f}, {py:.2f}, {pth:.2f}) RPM({rl:.1f}, {rr:.1f})"
                                    log_file.write(f"{timestamp};RX;{decoded}\n")
                                else:
                                    log_file.write(f"{timestamp};RX;{line}\n")
                            except ValueError:
                                log_file.write(f"{timestamp};RX;{line} (Parse Error)\n")
                        else:
                            log_file.write(f"{timestamp};RX;{line}\n")
                        log_file.flush()
                except UnicodeDecodeError:
                    # If decoding fails, log raw bytes
                    log_file.write(f"{datetime.datetime.now()};RX_RAW;{raw_line}\n")
                    log_file.flush()
            else:
                time.sleep(0.01)
        except OSError:
            print("\nSerial port disconnected.")
            running = False
            break

def main():
    global running
    
    port = DEFAULT_PORT
    if len(sys.argv) > 1:
        port = sys.argv[1]

    try:
        print(f"Connecting to {port} at {BAUD_RATE} baud...")
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for Arduino to reset after connection
        print("Connected successfully.")
    except serial.SerialException as e:
        print(f"Error connecting to serial port: {e}")
        print(f"Hint: Make sure you have permission (sudo usermod -a -G dialout $USER) and the port is correct.")
        return

    log_filename = "log.txt"
    log_file = open(log_filename, 'w')
    print(f"\nLogging all serial activity to: {log_filename}")
    print(f"View with: tail -f {log_filename}\n")

    # Start the receiver thread
    rx_thread = threading.Thread(target=receive_data, args=(ser, log_file), daemon=True)
    rx_thread.start()

    print("\n--- QBot Serial Test Node ---")
    print("Commands:")
    print("  m <mode>    -> Set Mode (0=IDLE, 1=VELOCITY, 2=DIRECT, 3=ACTION, 4=DIAGNOSTIC)")
    print("  v <l> <a>   -> Set Velocity (linear m/s, angular rad/s) [Mode 1 only]")
    print("                 Ex: v 0.2 0.0  (Forward)")
    print("  d <l> <r>   -> Direct Drive PWM (-255 to 255) [Mode 2 only]")
    print("                 Ex: d 100 100  (Forward)")
    print("  s           -> Stop (Force IDLE)")
    print("  r           -> Reset Odometry (X=0, Y=0, Theta=0)"ls
    print("  t <mode>    -> Set Telemetry (0=OFF, 1=TEXT, 2=CSV)")
    print("Type 'exit' to quit.\n")

    try:
        while running:
            # Get user input
            user_input = input("TX: ")
            
            if user_input.lower() == 'exit':
                break
            
            if user_input:
                # Log the command being sent
                log_file.write(f"{datetime.datetime.now()};TX;{user_input}\n")
                log_file.flush()
                # Send to Arduino
                ser.write(user_input.encode('utf-8'))
                
    except (KeyboardInterrupt, EOFError):
        print("\nExiting...")
    finally:
        running = False
        if ser.is_open:
            ser.close()
        log_file.close()
        print("Log file closed.")

if __name__ == "__main__":
    main()