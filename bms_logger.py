import logging
import time
import random

# === Logging Configuration ===
logging.basicConfig(
    filename="system_bms.log",
    filemode="a",
    level=logging.INFO,
    format="%(asctime)s | Voltage: %(voltage).2f V | Current: %(current).2f A | Power: %(power).2f W"
)

def get_bms_data():
    """
  Will  Replace this function with real BMS reading logic.
    For example, using serial/I2C communication with our BMS hardware.
    """
    voltage = round(random.uniform(11.5, 12.6), 2)  # in volts
    current = round(random.uniform(0.8, 2.5), 2)    # in amperes
    power = round(voltage * current, 2)             # watts (P = V × I)
    return voltage, current, power

def log_bms_data():
    """Continuously log BMS parameters to a system log file."""
    while True:
        voltage, current, power = get_bms_data()
        logging.info("", extra={"voltage": voltage, "current": current, "power": power})
        print(f"🔋 Logged BMS Data → V={voltage}V, I={current}A, P={power}W")
        time.sleep(5)

if __name__ == "__main__":
    log_bms_data()

