#!/usr/bin/env python3
import subprocess
import socket

def setup_adb_reverse():
    """
    Sets up ADB reverse so that TCP connections on port 9000 on the Quest
    are forwarded to port 9000 on the Linux PC.
    """
    try:
        result = subprocess.run(
            ["adb", "reverse", "tcp:8080", "tcp:8080"],
            capture_output=True,
            text=True
        )
        if result.returncode != 0:
            print("ADB reverse failed:", result.stderr)
        else:
            print("ADB reverse successful:", result.stdout.strip())
    except Exception as e:
        print("Error setting up ADB reverse:", e)

def start_tcp_server(host='0.0.0.0', port=8080):
    """
    Starts a TCP server that listens on the specified host and port.
    It accepts incoming connections and prints any data received.
    """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"TCP server listening on {host}:{port}")

    try:
        while True:
            print("Waiting for an incoming connection...")
            client_socket, addr = server_socket.accept()
            print(f"Accepted connection from {addr}")

            try:
                while True:
                    data = client_socket.recv(1024)
                    if not data:
                        print("Client closed the connection.")
                        break
                    print("Received message:", data.decode('utf-8'))
            except Exception as e:
                print("Error during data reception:", e)
            finally:
                client_socket.close()
    except KeyboardInterrupt:
        print("Server interrupted by user. Shutting down.")
    finally:
        server_socket.close()

if __name__ == "__main__":
    setup_adb_reverse()
    start_tcp_server()
