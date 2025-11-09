# generate_fake_ir_frames.py
import cv2
import numpy as np
import os

os.makedirs("data", exist_ok=True)

def generate_ir_frame(width=640, height=480, peaks=None, filename="data/sample.png"):
    """
    Generate a fake IR sensor frame.
    `peaks` is a list of (center, width, depth) tuples defining dips (absorption features).
    """
    y = np.linspace(0, height - 1, height)
    x = np.linspace(0, width - 1, width)
    xx, yy = np.meshgrid(x, y)

    # base bright stripe: Gaussian centered vertically
    band_center = height // 2
    band_sigma = height / 20
    band_intensity = np.exp(-0.5 * ((yy - band_center) / band_sigma) ** 2)

    # start from bright 200 intensity (out of 255)
    line = np.ones(width) * 200.0

    # apply synthetic dips
    if peaks:
        for c, w, d in peaks:
            dip = d * np.exp(-0.5 * ((x - c) / w) ** 2)
            line -= dip

    # normalize to 0–255
    line = np.clip(line, 0, 255)
    # replicate vertically across the band
    frame = band_intensity * line
    frame = (255 * frame / frame.max()).astype(np.uint8)

    # optional Gaussian blur to make it realistic
    frame = cv2.GaussianBlur(frame, (7, 7), 0)
    cv2.imwrite(filename, frame)
    print(f"[OK] Saved {filename}")


# --- generate few samples ---
generate_ir_frame(peaks=[(150, 20, 40), (500, 30, 30)], filename="data/sample1.png")
generate_ir_frame(peaks=[(200, 50, 60), (400, 30, 40)], filename="data/sample2.png")
generate_ir_frame(peaks=[], filename="data/sample3.png")  # flat (inorganic)
generate_ir_frame(peaks=[(550, 25, 80)], filename="data/sample4.png")

