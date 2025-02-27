import socket

UDP_IP = "0.0.0.0"  # Listen on all available interfaces.
UDP_PORT = 9000     # Must match the port used in your headset script.

# Create a UDP socket.
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(4096)  # Buffer size of 4096 bytes.
    message = data.decode('utf-8')
    print(f"Received from {addr}: {message}")
