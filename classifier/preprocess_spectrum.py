# preprocess_spectrum.py
import pandas as pd
import numpy as np
from scipy.signal import savgol_filter, find_peaks

def validate_spectrum(spectrum): #data validation
    #Validate and clean the raw spectrum data.
    #removes non umeric values and duplicates, and sorts by wavenumber.
    df = pd.DataFrame(spectrum)
    df = df.dropna().drop_duplicates()
    df = df[df['wavenumber'].between(400, 4000)]  # typical IR range
    df = df.sort_values('wavenumber', ascending=False).reset_index(drop=True)
    return df

def smooth_spectrum(df, window_length=11, polyorder=3): #data smoothing
    #apply savitzkygolay filter to smoothen signal
    t = df['transmittance'].values
    if len(t) >= window_length:
        df['transmittance_smooth'] = savgol_filter(t, window_length, polyorder)
    else:
        df['transmittance_smooth'] = t  # skip smoothing for small sets
    return df

def normalize_spectrum(df): #normalise data

    #normalize transmittance values to 0–100 range to ensure stable peak thresholds for different intensity scales
    t = df['transmittance_smooth']
    df['transmittance_norm'] = 100 * (t - t.min()) / (t.max() - t.min() + 1e-8)
    return df


def detect_peaks(df, prominence=1.0, distance=10): #peak detection
    #detects absorbance maxima by inverting the transmittance curve.
    inverted = 100 - df['transmittance_norm'] # invert signal because IR peaks are minima in %T
    peaks, props = find_peaks(inverted, prominence=prominence, distance=distance)

    peak_table = df.iloc[peaks][['wavenumber', 'transmittance_norm']]
    peak_table = peak_table.rename(columns={'transmittance_norm': 'transmittance'})
    return peak_table.reset_index(drop=True)


def preprocess_spectrum(raw_spectrum): #main function
    df = validate_spectrum(raw_spectrum)
    df = smooth_spectrum(df)
    df = normalize_spectrum(df)
    peaks = detect_peaks(df)
    return peaks


def save_peak_table(peaks, filename="detected_peaks.csv"): #saves values in csv
    peaks.to_csv(filename, index=False)
    print(f"[INFO] Saved {len(peaks)} peaks to {filename}")

