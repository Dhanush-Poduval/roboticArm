python
import os
import time

def check_connection(target_ip="192.168.88.1"):
    """Ping target IP to verify network connectivity."""
    response = os.system(f"ping -c 1 {target_ip} > /dev/null 2>&1")
    return response == 0

if __name__ == "__main__":
    while True:
        status = check_connection()
        print(" Connected" if status else "Disconnected")
        time.sleep(5)
