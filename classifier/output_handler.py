# output_handler.py

import csv
import os
from datetime import datetime

def log_result(result, logfile="classification_log.csv"): #writes to csv log file
    file_exists = os.path.isfile(logfile)
    with open(logfile, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["timestamp", "category", "elements", "groups"])
        writer.writerow([
            result.get("timestamp", datetime.now().isoformat(timespec='seconds')),
            result.get("category", "NA"),
            ",".join(result.get("detected_elements", [])),
            ",".join(result.get("detected_groups", []))
        ])
    print(f"[LOG] Saved result to {logfile}")

def send_to_arm():
    #placeholder function, will coordinate w Dhanush and fix this
    pass
