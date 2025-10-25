# main.py

import time
import traceback
from datetime import datetime

from read_raw_spectrum import read_raw_spectrum
from preprocess_spectrum import preprocess_spectrum
from ir_classifier import classify_sample
from output_handler import log_result, send_to_arm

def setup():
    print("=" * 60)
    print("IR-BASED CLASSIFICATION SYSTEM - MAIN CONTROLLER")
    print(f"Startup time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)
    print("System is initializing modules...")
    time.sleep(1)
    print("Modules loaded successfully.\n")


def process_sample():
    try:
        raw = read_raw_spectrum()
        if not raw or len(raw) == 0:
            print("[WARN] Empty sensor data received. Skipping cycle.")
            return

        peaks = preprocess_spectrum(raw)
        if peaks.empty:
            print("[WARN] No peaks detected in current spectrum.")
            return

        result = classify_sample(peaks)
        print(f"[RESULT] Category: {result['category']} | Elements: {','.join(result['detected_elements'])}")

        log_result(result)
        #will add this function later once arm interface is figured out
        #send_to_arm(result)

    except Exception as e:
        print(f"[ERROR] {e}")
        traceback.print_exc()
        time.sleep(1)


def main_loop(delay=2):
    setup()
    print(f"System running... Press Ctrl+C to stop.\n")

    try:
        while True:
            process_sample()
            time.sleep(delay)  # wait before next measurement

    except KeyboardInterrupt:
        print("System stopped manually by user.")
    except Exception as e:
        print(f"[FATAL ERROR] {e}")
        traceback.print_exc()
    finally:
        print("Cleanup complete. Exiting safely.")


if __name__ == "__main__":
    main_loop(delay=2)

