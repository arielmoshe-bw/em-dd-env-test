import serial
import matplotlib.pyplot as plt
from collections import deque
import time

# Initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 115200)

time.sleep(4)

# Create deque objects to store data
maxlen_samples = 50  # Number of samples to save
data1 = deque(maxlen=maxlen_samples)  # For Current in amps
data2 = deque(maxlen=maxlen_samples)  # For Encoder velocity in amps
data3 = deque(maxlen=maxlen_samples)  # For Encoder position in amps

# Flag to indicate whether the first sample has been received
first_sample_received = False

# Set up the plot
plt.ion()
fig, axs = plt.subplots(3, 1)
lines = []

for i in range(3):
    lines.append(axs[i].plot([], [], lw=2)[0])

axs[0].set_title('Current in amps')
axs[1].set_title('Encoder velocity in rpm')
axs[2].set_title('Encoder position in degree')
plt.tight_layout()

# Function to update the plot
# Initialize fault code
fault_code = ""


# Function to update the plot and display fault code as text
def update_plot():
    lines[0].set_data(range(len(data1)), data1)
    lines[1].set_data(range(len(data2)), data2)
    lines[2].set_data(range(len(data3)), data3)
    for ax in axs:
        ax.relim()
        ax.autoscale_view()

    # Display fault code as text below the graphs
    axs[-1].text(0.5, -0.25, "Fault Code: " + fault_code, horizontalalignment='center', verticalalignment='center',
                 transform=axs[-1].transAxes)

    plt.draw()
    plt.pause(0.01)  # Adjust the pause duration as needed


# Main loop to collect data and update the plot
while True:
    # Read a line from serial as raw bytes
    line = ser.readline()

    # Decode the line as UTF-8, ignoring errors
    try:
        line = line.decode('utf-8').strip()
    except UnicodeDecodeError:
        print("Failed to decode line:", line)
        continue

    # Skip processing if the line is empty
    if not line:
        continue

    # Split the line by ':' to separate label and value
    parts = line.split(':')
    if len(parts) != 2:
        print("Invalid data format:", line)
        continue

    label, value = parts

    # Check if it's a fault code update
    if label == "Fault Code":
        fault_code = value  # Update fault_code variable here
        print("Received fault code:", fault_code)  # Print received fault code for debugging
        update_plot()
        continue

    # Skip the first sample
    if not first_sample_received:
        first_sample_received = True
        continue

    # Append data to respective deques based on label
    if label == "Current in amps":
        data1.append(float(value))
    elif label == "Encoder velocity in rpm":
        data2.append(float(value))
    elif label == "Encoder position in degree":
        data3.append(int(value))

    # Update the plot
    update_plot()
