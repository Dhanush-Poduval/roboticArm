import warnings 
import torch
import cv2
import numpy as np
import math
import json
import os
import time
import sys 
warnings.filterwarnings("ignore", category=FutureWarning)

print("Loading YOLOv5 model. This may take a moment...")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("CRITICAL ERROR: Failed to open camera (cv2.VideoCapture(0)).")
    sys.exit(1) 

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

unique_centers = [] 
tolerance = 50 
max_containers = 5 
detected_aruco_positions = {}
container_positions = {}
run_time_s = 5 
camera_matrix = np.array([[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float32)
dist = np.zeros((4, 1), dtype=np.float32)

fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2] 
cy = camera_matrix[1, 2] 




def get_simulated_tof_distance(x_pixel, y_pixel):
    return 0.5 

def calculated_3d_coods(u, v, z_tof_meters):
    """Calculates Xc, Yc, Zc in MILLIMETERS (mm)."""
    Zc_mm = z_tof_meters * 1000.0
    Xc_mm = (u - cx) * Zc_mm / fx
    Yc_mm = (v - cy) * Zc_mm / fy

    return Xc_mm, Yc_mm, Zc_mm
start_time = time.time()
print("\nRunning vision for 5s stabilization. Look at the 'Vision Processing' window.")

while (time.time() - start_time) < run_time_s: 
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame from camera.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco_detector.detectMarkers(gray)
    detected_aruco_positions = {} 
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(ids)): 
            marker_id = ids[i][0]
            c = corners[i][0]

            marker_cx = int((c[0][0] + c[2][0]) / 2)
            marker_cy = int((c[0][1] + c[2][1]) / 2)
            detected_aruco_positions[marker_id] = (marker_cx, marker_cy)
            
            cv2.circle(frame, (marker_cx, marker_cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"Aruco ID: {marker_id}", (marker_cx + 10, marker_cy - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    results = model(frame)
    
    for *box, conf, cls in results.xyxy[0]:
        x1, y1, x2, y2 = box
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cx_yolo = int((x1 + x2)/2) 
        cy_yolo = int((y1 + y2)/2)
        
        if all(math.hypot(cx_yolo - ux, cy_yolo - uy) > tolerance for ux, uy in unique_centers):
            unique_centers.append((cx_yolo, cy_yolo))
    container_positions = {}
    
    for i, (ux, uy) in enumerate(unique_centers, 1):
        cv2.circle(frame, (ux, uy), 5, (0, 0, 255), -1)
        
        z_tof_meters = get_simulated_tof_distance(ux, uy)
        xc, yc, zc = calculated_3d_coods(ux, uy, z_tof_meters)
        container_positions[f'C{i}'] = (round(xc, 2), round(yc, 2), round(zc, 2))
        text = f"C{i}: X:{xc:.2f}, Y:{yc:.2f}, Z:{zc:.2f} (mm)"
        cv2.putText(frame, text, (ux - 50, uy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.imshow("Vision Processing", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Quit key pressed. Exiting loop.")
        break

cap.release()
cv2.destroyAllWindows()

print("\n--- Final Vision Output ---")
print("Container 3D Positions (mm):", container_positions)
print("AruCo Pixel Positions:", detected_aruco_positions)
output_data = {
    'containers': container_positions,
    'aruco_pixels': detected_aruco_positions 
}
json_filepath = 'vision_data.json'
with open(json_filepath, 'w') as f:
    json.dump(output_data, f, indent=4)

print(f"\nData successfully saved to {json_filepath}")