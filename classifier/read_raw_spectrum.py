# Placeholder sensor reader module for IR-based classifier system
"""
read_raw_spectrum.py
---------------------------------
This module represents the interface between the spectroscopy sensor and
the classification system.

For now, it runs in a simulation mode, where a grayscale image
(simulated frame of the diffraction pattern) is read and converted into
a 1-D spectrum.

When the real hardware (e.g. ESP32 + OV7670) is available, it can be integrated using the capture_spectrum function

Outputs:
    list of dicts like:
    [
        {"wavenumber": 3999.5, "transmittance": 82.4},
        {"wavenumber": 3998.7, "transmittance": 84.1},
        ...
    ]
"""

import cv2
import numpy as np
from typing import List, Dict, Optional
import time

# ---------------------------------------------------------------------
# 1. Acquisition lifecycle controls
# ---------------------------------------------------------------------

def start_acquisition():
    """Simulate powering up or initializing the spectrometer."""
    print("[ACQ] Sensor initializing...")
    time.sleep(1.0)
    print("[ACQ] Sensor ready for capture.")


def stop_acquisition():
    """Simulate shutting down or releasing resources."""
    print("[ACQ] Stopping sensor...")
    time.sleep(0.5)
    print("[ACQ] Sensor standby.\n")


# ---------------------------------------------------------------------
# 2. Spectrum extraction pipeline
# ---------------------------------------------------------------------

def _to_gray(frame: np.ndarray) -> np.ndarray:
    """Convert any image to grayscale float64."""
    if frame.ndim == 2:
        gray = frame.astype(np.float64)
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float64)
    return gray


def _find_bright_band(gray: np.ndarray, band_height: int = 9) -> np.ndarray:
    """Locate the brightest horizontal band and average a few rows."""
    row_energy = gray.mean(axis=1)
    y_center = int(np.argmax(row_energy))
    half = max(1, band_height // 2)
    y0, y1 = max(0, y_center - half), min(gray.shape[0], y_center + half + 1)
    line = gray[y0:y1, :].mean(axis=0)
    return line


def _pixel_to_wavenumber(n_pixels: int,
                         wn_max: float = 4000.0,
                         wn_min: float = 400.0) -> np.ndarray:
    """Map pixel index → wavenumber (cm⁻¹) linearly (placeholder)."""
    return np.linspace(wn_max, wn_min, n_pixels, dtype=np.float64)


def _normalize_to_percent(intensity: np.ndarray) -> np.ndarray:
    """Normalize arbitrary intensities to 0–100% range (proxy for %T)."""
    if intensity.size == 0:
        return intensity
    maxv = float(intensity.max())
    return 100.0 * (intensity / (maxv + 1e-8))


# ---------------------------------------------------------------------
# 3. Core acquisition function (single image per sample)
# ---------------------------------------------------------------------

def capture_spectrum(image_path: str,
                     band_height: int = 9,
                     wn_max: float = 4000.0,
                     wn_min: float = 400.0) -> List[Dict[str, float]]:
    """
    Simulate capturing one spectrum frame and converting it to data.

    Args:
        image_path : path to the simulated sensor image (PNG/JPG)
        band_height : number of rows to average for noise reduction
        wn_max, wn_min : mapping range for wavenumber (cm⁻¹)

    Returns:
        list of dicts [{wavenumber, transmittance}, ...]
    """

    print(f"[ACQ] Capturing frame from {image_path} ...")

    # -----------------------------------------------------------------
    # --- Real hardware capture block (to be implemented later) -------
    # -----------------------------------------------------------------
    """
    # Example placeholder for real ESP32 + OV7670 sensor read:
    import requests
    response = requests.get("http://<esp32_ip>/capture", timeout=3)
    arr = np.frombuffer(response.content, np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("Failed to read frame from sensor.")
    """

    # -----------------------------------------------------------------
    # --- Simulation: read frame from saved image ---------------------
    # -----------------------------------------------------------------
    frame = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    if frame is None:
        raise FileNotFoundError(f"[ERROR] Could not read image: {image_path}")

    gray = _to_gray(frame)
    line = _find_bright_band(gray, band_height=band_height)
    wn = _pixel_to_wavenumber(len(line), wn_max, wn_min)
    T = _normalize_to_percent(line)

    spectrum = [{"wavenumber": float(w), "transmittance": float(t)}
                for w, t in zip(wn, T)]

    print(f"[ACQ] Spectrum extracted: {len(spectrum)} points.")
    return spectrum

