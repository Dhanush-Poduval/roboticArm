import torch
import cv2
import math
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
cap = cv2.VideoCapture(0)
unique_centers = [] 
tolerance = 10 
max_containers = 5 

while True:
    ret, frame = cap.read()
    if not ret:
        break

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
