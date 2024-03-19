import os
import sys
import serial
import csv
import signal
import matplotlib.pyplot as plt
from collections import deque
import time
import threading
from queue import Queue

error_descriptions = [
    "Disabled",
    "Overvoltage",
    "Hardware protection",
    "E2PROM",
    "Undervoltage",
    "N/A",
    "Overcurrent",
    "Mode failure",
    "Less phase",
    "Motor stall",
    "Reserved",
    "Hall failure",
    "Current sensing",
    "232 disconnected",
    "CAN disconnected",
    "Motor stalled"
]
# Initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 115200)

# Wait for "start sampling" message from Arduino
while True:
    ser.write(b"ready\n")
    line = ser.readline().decode('utf-8').strip()
    if line == "start sampling":
        break

# Create deque objects to store data
maxlen_samples = 50  # Number of samples to save
data1 = deque(maxlen=maxlen_samples)  # For Current in amps
data2 = deque(maxlen=maxlen_samples)  # For Voltage in volts
data3 = deque(maxlen=maxlen_samples)  # For Temperature in degree
data4 = deque(maxlen=maxlen_samples)  # For Encoder velocity in rpm
data5 = deque(maxlen=maxlen_samples)  # For Encoder position in degree

# Set up the plot
plt.ion()
fig, axs = plt.subplots(5, 1)
lines = []

for i in range(5):
    lines.append(axs[i].plot([], [], lw=2)[0])

axs[0].set_title('Current in amps', fontsize=15, weight='bold')
axs[1].set_title('Voltage in volts', fontsize=15, weight='bold')
axs[2].set_title('Temperature in degree', fontsize=15, weight='bold')
axs[3].set_title('Encoder velocity in rpm percentages (-1000, 1000)', fontsize=15, weight='bold')
axs[4].set_title('Encoder position in steps (-50k, 50k) 5 circles cw & 5 circles ccw', fontsize=15, weight='bold')
plt.tight_layout()

# Initialize fault code
fault_code = 0
fault_code_text = ""

stop_plotting = False

# Queue for communication between threads
data_queue = Queue()

# Initialize time variables
start_time = time.time()


# Function to update the plot and display fault code as text
def update_plot():
    global start_time

    while not data_queue.empty():
        item = data_queue.get()
        if item is None:
            continue
        label, value = item
        if label == "Fault code":
            global fault_code
            fault_code = int(value, 16)

        elif label == "Current in amps":
            data1.appendleft(float(value))
        elif label == "Voltage in volts":
            data2.appendleft(float(value))
        elif label == "Temperature in degree":
            data3.appendleft(int(value))
        elif label == "Encoder velocity in rpm":
            data4.appendleft(float(value))
        elif label == "Encoder position in degree":
            data5.appendleft(int(value))

    elapsed_time = time.time() - start_time

    x_axis1 = [elapsed_time - j * 0.005 for j in range(len(data1))]
    x_axis2 = [elapsed_time - j * 0.005 for j in range(len(data2))]
    x_axis3 = [elapsed_time - j * 0.005 for j in range(len(data3))]
    x_axis4 = [elapsed_time - j * 0.005 for j in range(len(data4))]
    x_axis5 = [elapsed_time - j * 0.005 for j in range(len(data5))]

    for k, line in enumerate(lines):
        if k == 0:
            line.set_data(x_axis1, data1)
        elif k == 1:
            line.set_data(x_axis2, data2)
        elif k == 2:
            line.set_data(x_axis3, data3)
        elif k == 3:
            line.set_data(x_axis4, data4)
        elif k == 4:
            line.set_data(x_axis5, data5)

    for ax in axs:
        ax.set_xlabel('Time (s)')
        ax.relim()
        ax.autoscale_view()

    if not axs[-1].texts:
        fault_text = axs[-1].text(0.5, 14.3, "", horizontalalignment='center', verticalalignment='center',
                                  transform=axs[-1].transAxes, fontsize=15, weight='bold')
    else:
        fault_text = axs[-1].texts[0]

    # Extract and display error descriptions from fault code
    active_errors = [error_descriptions[i] for i in range(16) if (fault_code >> i) & 1]
    global fault_code_text
    fault_code_text = "Fault code: " + hex(fault_code) + " - [ " + ", ".join(active_errors) + " ]"
    fault_text.set_text(fault_code_text)

    plt.draw()
    plt.pause(0.0005)


# Function for data acquisition
def data_acquisition():
    global stop_plotting
    while not stop_plotting:
        line = ser.readline()
        try:
            line = line.decode('utf-8').strip()
        except UnicodeDecodeError:
            print("Failed to decode line:", line)
            continue

        if not line:
            continue

        parts = line.split(':')
        if len(parts) != 2:
            print("Invalid data format:", line)
            continue

        label, value = parts
        data_queue.put((label, value))


# Function to periodically save data to CSV file
def save_to_csv(start_time, stop_saving_event):
    try:
        with open('data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                ['Time (s)', 'Current (amps)', 'Voltage (volts)', 'Temperature (degree)', 'Encoder velocity (rpm)',
                 'Encoder position (degree)', 'Error Description'])

            while not stop_saving_event.is_set():
                # Collect data from all queues (assuming same length)
                data_points = []
                if len(data1) > 0:
                    for queue in [data1, data2, data3, data4, data5]:
                        data_points.append(queue[-1])  # Get the latest value from each queue
                    global fault_code_text
                    data_point = (time.time() - start_time,) + tuple(data_points) + (fault_code_text,)
                    writer.writerow(data_point)
                time.sleep(0.1)  # Adjust the interval as needed

    except Exception as e:
        print("Error occurred while writing to CSV:", e)
    finally:
        csvfile.close()


# Function to handle interrupt signal (Ctrl+C)
def signal_handler(sig, frame):
    print("Script stopped")
    stop_saving_event.set()  # Signal the save_to_csv thread to stop

    global stop_plotting  # Access the global flag
    stop_plotting = True


# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# Create a stop event object for the save_to_csv thread
stop_saving_event = threading.Event()

# Start the save to CSV thread with start_time and stop_event as arguments
csv_thread = threading.Thread(target=save_to_csv, args=(start_time, stop_saving_event))
csv_thread.start()

# Start the data acquisition thread
data_thread = threading.Thread(target=data_acquisition)
data_thread.start()

# Main loop to update the plot
while not stop_plotting:
    update_plot()

plt.close()
os._exit(1)
