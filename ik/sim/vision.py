import pybullet as p
import numpy as np
import cv2
import math
from scipy.spatial.transform import Rotation as R

camera_width=640
camera_height=480
FOV=60.0
near_plane=0.01
far_plane=10.0

focal_length=(camera_width/2)/math.tan(math.radians(FOV/2))

K=np.array([
    [focal_length,0,camera_width/2],
    [0,focal_length,camera_height/2],
    [0,0,1]

],dtype=np.float32)

dist_coeffs=np.zeros((4,1)) #just to make it smooth in sim

def get_containers(client_id,container_color,camera_eye,camera_target):
    View_matrix=p.computeViewMatrix(
        cameraEyePosition=camera_eye,
        cameraTargetPosition=camera_target,
        cameraUpVector=[0,0,1]
    )
    projection_matrix=p.computeProjecttionMatrixFOV(
        fov=FOV,aspect=camera_width/camera_height,nearVal=near_plane,farVal=far_plane
    )

    img_arr=p.getCameraImage(
        camera_width,
        camera_height,
        viewMatrix=View_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    rgb_data=np.reshape(img_arr[2],(camera_height,camera_width,4))[:,:,:3]
    depth_data = np.reshape(img_arr[3], (camera_height, camera_width))#simulating the tof sensor data basically

    bgr_image=cv2.cvtColor(rgb_data,cv2.COLOR_RGB2BGR)