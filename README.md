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

├── /classifier
│   └──ir_classifier.py
│   └── main.py
│   └── output_handler.py
|   └── preprocess_spectrum.py
|   └── read_raw_spectrum.py
├── ik/
│   └── dh.py
│   └── motor_control.py
|   └── fk.py
|   └── four_dof_arm.py
|   ├── sim
|     └──sim_ik.py
|     └── vision.py
|   ├── meshes
|     └──consists all the mesh detail file
|
├── main
├  └──check.py
├  └── inverse_k.py
├  └── main.py
└  └── arm_movement.py
|  └──callibration.py
├── README.md
├── requirements.txt
└── .gitignore
