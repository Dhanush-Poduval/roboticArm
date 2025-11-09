# main.py

import os
import time
import traceback
from read_raw_spectrum import start_acquisition, capture_spectrum, stop_acquisition
from preprocess_spectrum import preprocess_spectrum
from ir_classifier import classify_sample
from output_handler import log_result
# from output_handler import send_to_arm   #


def process_one(image_path: str)-> None:
    """
    Process a single sample (one image or one sensor reading).
    This simulates the workflow for one robotic arm cycle.
    """
    try:
        print(f"\n============================================================")
        print(f"Processing sample: {image_path}")
        print(f"============================================================")

        if not os.path.exists(image_path):
            print(f"[WARN] File not found: {image_path}")
            return
        start_acquisition()  # turn on / prepare the sensor
        raw = capture_spectrum(image_path)  # capture one spectrum frame
        peaks = preprocess_spectrum(raw)

        if hasattr(peaks, "empty") and peaks.empty:
            print("[WARN] No peaks detected in current spectrum.")
            stop_acquisition()
            return

        result = classify_sample(peaks)
        print(f"[RESULT] Category: {result['category']} | Elements: {','.join(result['detected_elements'])}")

        log_result(result)
        # send_to_arm(result)  # future integration

        stop_acquisition()  # safely stop the sensor

    except Exception as e:
        print(f"[ERROR] {e}")
        traceback.print_exc()
        stop_acquisition()
        time.sleep(1)


DEFAULT_SAMPLE = "data/1.png"

def repl():
    global CUR_IDX
    while True:
        try:
            cmd = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[•] Exiting.")
            break

        if not cmd:
            continue

        tokens = cmd.split()
        op = tokens[0].lower()

        if op in ("quit", "exit"):
            print("[•] END OF CLASSIFICATIONS.")
            break

        if op == "start":
            if len(tokens) == 1:
                if CUR_IDX >= len(sample_images):
                    print("[ERROR] No more files in sample_images. Use 'reset")
                    continue
                img = sample_images[CUR_IDX]
                CUR_IDX +=1
            else:
                if tokens[1].isdigit():
                    idx=int(tokens[1])-1
                    if 0<=idx < len(sample_images):
                        CUR_IDX = idx +1
                        img = sample_images[idx]
                    else:
                        print(f"[!] Index out of range. Valid: 1..{len(sample_images)}")
                else:
                    img = " ".join(tokens[1:])

            process_one(img)
            continue
        if op == "reset":
            CUR_IDX = 0
            print("[•] Reset. Next start will use:", sample_images[CUR_IDX] if sample_images else "-")
        print(f"[?] Unknown command: {op!r}. Type 'help' for options.")


if __name__ == "__main__":
    # List of sample images (simulated frames)
    sample_images = [
        "data/sample1.png",
        "data/sample2.png",
        "data/sample3.png",
        "data/sample4.png"
    ]
    CUR_IDX = 0

    print("\n============================================================")
    print("IR-BASED CLASSIFICATION SYSTEM - MAIN CONTROLLER")
    print(f"Startup time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("============================================================")

    repl()
