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
├── classifier
│   ├── .gitignore
│   ├── ir_classifier.py
│   ├── main.py
│   ├── output_handler.py
│   ├── preprocess_spectrum.py
│   └── read_raw_spectrum.py
├── communication
│   ├── command_listener.py
│   ├── data_sender.py
│   ├── diagnostics_logger.py
│   └── network_check.py
├── ik
├── m
│   ├── __pycache__
│   ├── yolov5
│   ├── arm_movement.py
│   ├── calibration.py
│   ├── check.py
│   ├── inverse_k.py
│   ├── main.py
│   ├── motion_control.py
│   ├── vision_data.json
│   └── yolov5s.pt
├── venv
├── yolov5
│   ├── .gitignore
│   ├── datasets.yaml
│   ├── detect.py
│   ├── setup.py
│   └── yolov5s.pt
├── .gitignore
├── datasets.yaml
├── detect.py
├── setup.py
└── yolov5s.pt
```

# How it works

## Classifier
- The classifier works on a rule based system that gets a diffraction frame from the sensor, which is then converted to a 1 Dimensional Spectrum
- The spectrum is then preprocessed.
- The meaningful chemical bands are then detected and mapped to the specific functional groups and elements required
- The rule based logic then assigns classifications to the sample based on the inputs of the life sciences domain.

## Communication
- The communication is done using a mikrotik lhg5 system with a two hop system because a two hop system ensures a NLOS.
- Then 4 codes are made according to the needs of the problem statement such as network_check.py(pinging the robot to check connection), data_sender.py, command_listener.py and diagnostics_logger.py.
- Then for a simulation of the network one code with 3 blocks are made each block representing one node i.e. the base , the relay and the robot.
- Any command we send to the base will be recieved by the robot through the relay using three terminals of same pc.

## Inverse Kinematics
- inverse kinematics has simulations and functions for the simulations to use
- the simulation uses inverse kinematics to caluculate the angles at which each motor should move to reach the target
- based on this movement the motors / joints are moved to a particular angle
- it also takes in path planning and if a blockage in the path then it aborts the whole mechanism and skips it
- the process is first to clearence position then to container approach position then to final position ,closing gripper , holding the container returning to clearence position , then to target shelf clearence then to target position and dropping the container returing to clearence position and then looping

## Main loop 
- main loop uses the ik functions motor movement and retry logic to move the arm in the same way as in the simulations
-  the process is first to clearence position then to container approach position then to final position ,closing gripper , holding the container returning to clearence position , then to target shelf clearence then to target position and dropping the container returing to clearence position and then looping
-  then retry logic runs the object detection model and then checks if that container coordinates are present and then accordingly if there then retrys if not then continues

