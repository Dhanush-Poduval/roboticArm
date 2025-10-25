
import os
import time

LOG_FILE = "diagnostics.log"
TARGET_IP = "192.168.88.1"

def get_latency():
    response = os.popen(f"ping -c 1 {TARGET_IP} | grep 'time='").read()
    if "time=" in response:
        latency = response.split("time=")[-1].split(" ")[0]
        return float(latency)
    return None

def log_diagnostics():
    while True:
        latency = get_latency()
        entry = f"{time.ctime()} | Latency: {latency if latency else 'N/A'} ms\n"
        with open(LOG_FILE, "a") as f:
            f.write(entry)
        print(entry.strip())
        time.sleep(10)

if __name__ == "__main__":
    log_diagnostics()

