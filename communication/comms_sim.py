import socket
import sys
import time
import random

# Common network settings
BASE_IP = "192.168.0.1"
BASE_PORT = 5000
RELAY_PORT = 5001
ROBOT_PORT = 5002


def base_station():
    """Simulates the control center (like PC or base station)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((BASE_IP, BASE_PORT))
    print("[BASE] Station running at port", BASE_PORT)

    while True:
        cmd = input("Enter command for robot (MOVE / STATUS / STOP): ").strip()
        if not cmd:
            continue
        s.sendto(cmd.encode(), (BASE_IP, RELAY_PORT))

        # Wait for robot response
        s.settimeout(5)
        try:
            data, _ = s.recvfrom(1024)
            print("[BASE] Received:", data.decode())
        except socket.timeout:
            print("[BASE] No response received.")


def relay_node():
    """Acts as a bridge between base and robot (Hop 1)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((BASE_IP, RELAY_PORT))
    print("[RELAY] Node active on port", RELAY_PORT)

    while True:
        data, addr = s.recvfrom(1024)
        msg = data.decode()

        if addr[1] == BASE_PORT:
            print("[RELAY] From BASE → Forwarding to ROBOT:", msg)
            s.sendto(msg.encode(), (BASE_IP, ROBOT_PORT))
        elif addr[1] == ROBOT_PORT:
            print("[RELAY] From ROBOT → Forwarding to BASE:", msg)
            s.sendto(msg.encode(), (BASE_IP, BASE_PORT))


def robot_node():
    """Simulates the robot endpoint (Hop 2)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((BASE_IP, ROBOT_PORT))
    print("[ROBOT] Node running at port", ROBOT_PORT)

    while True:
        data, addr = s.recvfrom(1024)
        cmd = data.decode().upper()
        print("[ROBOT] Received command:", cmd)

        if cmd == "STATUS":
            reply = f"Battery: {random.randint(60,100)}%, Temp: {round(random.uniform(22,35),1)}°C"
        elif cmd == "MOVE":
            reply = "Robot moving forward..."
        elif cmd == "STOP":
            reply = "Robot stopped."
        else:
            reply = "Unknown command."

        time.sleep(1)
        s.sendto(reply.encode(), (BASE_IP, RELAY_PORT))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 network_sim.py [base|relay|robot]")
        sys.exit(1)

    role = sys.argv[1].lower()

    if role == "base":
        base_station()
    elif role == "relay":
        relay_node()
    elif role == "robot":
        robot_node()
    else:
        print("Invalid role! Choose from: base, relay, robot")
