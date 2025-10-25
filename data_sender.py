
import socket
import json
import time
import random

SERVER_IP = "192.168.92.101"  # Base station IP
SERVER_PORT = 5005

def get_sensor_data():
    """Simulated sensor data."""
    return {
        "temperature": round(random.uniform(20, 35), 2),
        "voltage": round(random.uniform(11.8, 12.5), 2),
        "current": round(random.uniform(1.0, 2.5), 2),
        "timestamp": time.time()
    }

def send_data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        data = json.dumps(get_sensor_data())
        sock.sendto(data.encode(), (SERVER_IP, SERVER_PORT))
        print("📡 Data sent:", data)
        time.sleep(2)

if __name__ == "__main__":
    send_data()
