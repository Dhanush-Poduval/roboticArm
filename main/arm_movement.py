from .check import detected_aruco_positions,container_positions
from ..ik.motor_control import motor_movement
from ..ik.inverse_k import is_valid
#this will be the ik function for easy access
J1,J2,J3,J4=20,130,150,60
containers=container_positions.values()
ik_target_pos=list(containers)
print("Containers seen are ")
for i , c in enumerate(ik_target_pos):
    print(f"x={c[0]},y={c[1]:.2f},z={c[3]:.2f}")
def inverse_kinematics(J1,J2,J3,J4,):
    pass

#this will be the clearence position movement
def clearence_position():
    pass

#this will be the motor_control 
def motor_movement():
    pass

#retry logic

def retry():
    pass

#target position movement

def target_shelf():
    pass
#recall clearence position

#logging to nextjs

#uvicorn at end

