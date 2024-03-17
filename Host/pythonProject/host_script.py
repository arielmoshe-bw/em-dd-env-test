import serial
import matplotlib.pyplot as plt
from collections import deque
import time

# Initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 115200)

# Wait for "start sampling" message from Arduino
while True:
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

axs[0].set_title('Current in amps')
axs[1].set_title('Voltage in volts')
axs[2].set_title('Temperature in degree')
axs[3].set_title('Encoder velocity in rpm')
axs[4].set_title('Encoder position in degree')
plt.tight_layout()

# Initialize fault code
fault_code = ""


# Function to update the plot and display fault code as text
# Function to update the plot and display fault code as text
def update_plot():
    lines[0].set_data(range(len(data1)), data1)
    lines[1].set_data(range(len(data2)), data2)
    lines[2].set_data(range(len(data3)), data3)
    lines[3].set_data(range(len(data4)), data4)
    lines[4].set_data(range(len(data5)), data5)

    for ax in axs:
        ax.relim()
        ax.autoscale_view()

    # Check if the text object for fault code already exists
    if not axs[-1].texts:
        # Create a new text object for fault code
        fault_text = axs[-1].text(0.1, 13, "", horizontalalignment='center', verticalalignment='center',
                                  transform=axs[-1].transAxes)
    else:
        # Use the existing text object for fault code
        fault_text = axs[-1].texts[0]

    # Update only the fault code number
    fault_text.set_text("Fault code: " + fault_code)

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
    if label == "Fault code":
        fault_code = value  # Update fault_code variable here
        # print("Received fault code:", fault_code)  # Print received fault code for debugging
        update_plot()

    # Append data to respective deques based on label
    if label == "Current in amps":
        data1.append(float(value))
    elif label == "Voltage in volts":
        data2.append(float(value))
    elif label == "Temperature in degree":
        data3.append(int(value))
    elif label == "Encoder velocity in rpm":
        data4.append(float(value))
    elif label == "Encoder position in degree":
        data5.append(int(value))

    # Update the plot
    update_plot()
