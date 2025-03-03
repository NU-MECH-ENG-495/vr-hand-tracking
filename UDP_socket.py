import socket
import time
import matplotlib.pyplot as plt

UDP_IP = "0.0.0.0"  # Listen on all available interfaces.
UDP_PORT = 9000     # Must match the port used in your headset script.

# Create a UDP socket.
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

# Variables to calculate frequency
packet_count = 0
start_time = time.time()
last_frequency = 0.0

# Set up matplotlib for interactive plotting.
plt.ion()
fig, ax = plt.subplots()
ax.set_ylim(-180, 180)
ax.set_ylabel("Angle (degrees)")
ax.set_title("Right Hand Angles")

while True:
    data, addr = sock.recvfrom(4096)  # Buffer size of 4096 bytes.
    packet_count += 1
    message = data.decode('utf-8')
    print(f"Received from {addr}: {message}")
    
    # Check for frequency update every second.
    current_time = time.time()
    elapsed = current_time - start_time
    if elapsed >= 1.0:
        last_frequency = packet_count / elapsed
        print(f"Frequency: {last_frequency:.2f} packets per second")
        packet_count = 0
        start_time = current_time

    # Look for "Right hand Angles:" and parse the subsequent lines.
    if "Right hand Angles:" in message:
        try:
            # Extract the part after the header.
            data_part = message.split("Right hand Angles:")[1]
            lines = data_part.strip().splitlines()
            angles = {}
            for line in lines:
                if line.strip():
                    # Each line expected to be like "Label: value°"
                    parts = line.split(":")
                    if len(parts) >= 2:
                        label = parts[0].strip()
                        # Remove the degree symbol and any extra spaces.
                        value_str = parts[1].replace("°", "").strip()
                        angle = float(value_str)
                        angles[label] = angle
            # Update bar chart if any angles were found.
            if angles:
                labels = list(angles.keys())
                values = [angles[k] for k in labels]
                
                # Clear and update the current axis.
                ax.cla()
                ax.bar(labels, values)
                ax.set_ylim(-180, 180)
                ax.set_ylabel("Angle (degrees)")
                # Update title to include frequency information.
                ax.set_title(f"Right Hand Angles - Frequency: {last_frequency:.2f} pps")
                plt.setp(ax.get_xticklabels(), rotation=45, ha="right")
                plt.tight_layout()
                plt.pause(0.001)
        except Exception as e:
            print("Error parsing angles:", e)
