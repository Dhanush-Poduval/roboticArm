import torch
import cv2
import numpy as np
import math
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
cap = cv2.VideoCapture(0)

aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params=cv2.aruco.DetectorParameters()
unique_centers = [] 
tolerance = 10 
max_containers = 5 
detected_aruco_positions={}

#right now just putting dummy values
camera_matrix=np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]],dtype=np.float32)
dist=np.zeros((4,1))


while True:
    ret, frame = cap.read()
    if not ret:
        break
    corners , ids,rejected=cv2.aruco.detectMarkers(frame,aruco_dict,parameters=aruco_params)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame,corners,ids)
    for i in range(len(ids)):
        marker_id=ids[i][0]
        c=corners[i][0]

        marker_cx=int((c[0][0]+c[2][0])/2)
        marker_cy=int((c[0][1]+c[2][1])/2)
        detected_aruco_positions[marker_id]=(marker_cx,marker_cy)
        cv2.circle(frame,(marker_cx,marker_cy),5,(255,0,0),-1)
        cv2.putText(frame, f"Aruco ID: {marker_id}", (marker_cx + 10, marker_cy - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)



    results = model(frame)
    
    for *box, conf, cls in results.xyxy[0]:
        x1, y1, x2, y2 = box
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cx = int((x1 + x2)/2)
        cy = int((y1 + y2)/2)
        if all(math.hypot(cx - ux, cy - uy) > tolerance for ux, uy in unique_centers):
            unique_centers.append((cx, cy))
        if len(unique_centers) >= max_containers:
            break
    for (ux, uy) in unique_centers:
        cv2.circle(frame, (ux, uy), 5, (0, 0, 255), -1)

    cv2.imshow("YOLOv5 Webcam", frame)
    if len(unique_centers) >= max_containers:
        print("All container centers detected!")
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break



cap.release()
cv2.destroyAllWindows()

print("Unique container centers:")
for i, (cx, cy) in enumerate(unique_centers, 1):
    print(f"Container {i}: ({cx}, {cy})")
print('AruCo markers centers :')
if detected_aruco_positions:
    for marker_id , pos in detected_aruco_positions.items():
        print(f"Marker ID {marker_id}:({pos[0]},{pos[1]})")
else:
    print("No detected arUco markers")
