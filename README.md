# Robotic Arm
This is the main code base for the Dexter ,containes the communication , classifiaction and autonomous movement code base

## Instalation
```bash
python3 -m venv ven

source venv/bin/activate

pip install pybullet , serial , keyboard

```
## Running of code
Running the code urself
```python
cd ik
cd sim
python3 sim_ik.py
```
```python
cd main
python3 arm_movement.py
```
Object detection
```python
cd main
python3 check.py
```

## File structure

The structure of the whole code 
```
.
├── classifier/
│   ├── .gitignore
│   ├── ir_classifier.py
│   ├── main.py
│   ├── output_handler.py
│   ├── preprocess_spectrum.py
│   └── read_raw_spectrum.py
├── communication/
├── ik/
├── main/
│   ├── __pycache__/
│   ├── arm_movement.py
│   ├── calibration.py
│   ├── check.py
│   ├── inverse_k.py
│   ├── main.py
│   ├── motion_control.py
│   ├── vision_data.json
│   └── yolov5s.pt
├── venv/
├── yolov5/
│   ├── .gitignore
│   ├── datasets.yaml
│   ├── detect.py
│   ├── setup.py
│   └── yolov5s.pt
├── .gitignore
├── datasets.yaml
├── detect.py
├── setup.py
└── yolov5s.pt
```
