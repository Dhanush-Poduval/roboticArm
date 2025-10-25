import socket
import json

HOST = "0.0.0.0"
PORT = 6000

def process_command(cmd):
    """Stub function – replace with actual motor control logic."""
    print(f"🎮 Executing command: {cmd}")

def listen_for_commands():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    print(f"Listening for commands on {HOST}:{PORT}")

    while True:
        data, _ = sock.recvfrom(1024)
        try:
            command = json.loads(data.decode())
            process_command(command)
        except Exception as e:
            print("Invalid command:", e)

if __name__ == "__main__":
    listen_for_commands()
