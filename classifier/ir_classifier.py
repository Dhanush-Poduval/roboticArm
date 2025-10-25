#ir_classifier.py

import pandas as pd
from datetime import datetime

FUNCTIONAL_GROUPS = [
    ("O-H", (3200, 3600), ["O"]),
    ("N-H", (3300, 3500), ["N"]),
    ("C=O", (1650, 1750), ["O"]),
    ("C-O", (1000, 1300), ["O"]),
    ("C-H", (2850, 3000), ["C", "H"]),
    ("C≡N", (2210, 2260), ["N"]),
    ("P=O", (1150, 1250), ["P"]),
    ("P-O", (900, 1100), ["P"]),
    ("S=O", (1000, 1100), ["S"]),
    ("C=S", (1050, 1200), ["S"]),
]

def detect_elements_from_peaks(peaks_df, tol=20):
    detected_elements = set()
    detected_groups = set()

    for _, row in peaks_df.iterrows():
        wn = row['wavenumber']
        for group_name, (low, high), elems in FUNCTIONAL_GROUPS:
            if (low - tol) <= wn <= (high + tol):
                detected_groups.add(group_name)
                detected_elements.update(elems)
    return detected_elements, detected_groups

def classify_sample(peaks):
    elems, groups = detect_elements_from_peaks(peaks)
    organic = ("C" in elems) and ("H" in elems)
    chonps_present = any(e in elems for e in ["C","H","O","N","P","S"])

    if organic and any(e in elems for e in ["O","N","P","S"]):
        category = 1
    elif organic and elems == {"C","H"}:
        category = 2
    elif not organic and chonps_present:
        category = 3
    else:
        category = 4

    result = {
        "timestamp": datetime.now().isoformat(timespec='seconds'),
        "category": category,
        "detected_elements": sorted(elems),
        "detected_groups": sorted(groups)
    }
    return result

