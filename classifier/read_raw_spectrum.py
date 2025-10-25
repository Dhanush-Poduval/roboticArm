# Placeholder sensor reader module for IR-based classifier system

def read_raw_spectrum():
    """
    Reads Ir spectra from the sesnor

    Returns:
        list of dicts with 'wavenumber' and 'transmittance' key value pairs like given below dummy data
        ]
    """
    # ---- Placeholder for system arch & skeleton of workflow, will have to be updated once we get sensor hardware details from ls team
    # For now, return dummy spectrum data
    spectrum = [
        {"wavenumber": 3240, "transmittance": 46},
        {"wavenumber": 2950, "transmittance": 10},
        {"wavenumber": 1650, "transmittance": 54},
        {"wavenumber": 1323, "transmittance": 75},
        {"wavenumber": 950,  "transmittance": 49}
    ]
    return spectrum

