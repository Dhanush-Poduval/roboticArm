# main.py
# ============================================
# IR-BASED CLASSIFICATION SYSTEM - MAIN CONTROLLER
# ============================================

import time
import traceback
from read_raw_spectrum import start_acquisition, capture_spectrum, stop_acquisition
from preprocess_spectrum import preprocess_spectrum
from ir_classifier import classify_sample
from output_handler import log_result
# from output_handler import send_to_arm   # uncomment when ready


def process_one(image_path):
    """
    Process a single sample (one image or one sensor reading).
    This simulates the workflow for one robotic arm cycle.
    """
    try:
        print(f"\n============================================================")
        print(f"Processing sample: {image_path}")
        print(f"============================================================")

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
        time.sleep(1)  # small pause before next sample

    except Exception as e:
        print(f"[ERROR] {e}")
        traceback.print_exc()
        stop_acquisition()
        time.sleep(1)


def process_many(image_paths):
    """
    Simulate multiple samples being analyzed sequentially.
    (Like the robotic arm bringing one sample at a time.)
    """
    for img in image_paths:
        process_one(img)

"""from preprocess_spectrum import plot_spectrum

peaks = preprocess_spectrum(raw)
plot_spectrum(peaks, title=f"Spectrum for {image_path}")
"""

if __name__ == "__main__":
    # List of sample images (simulated frames)
    sample_images = [
        "data/sample1.png",
        "data/sample2.png",
        "data/sample3.png"
    ]

    print("\n============================================================")
    print("IR-BASED CLASSIFICATION SYSTEM - MAIN CONTROLLER")
    print(f"Startup time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("============================================================")

    print("\nSystem initialized. Starting multi-sample processing...\n")
    process_many(sample_images)

    print("\nAll samples processed successfully. System exiting.\n")

