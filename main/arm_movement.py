from check import detected_aruco_positions,container_positions
from ..ik.motor_control import motor_movement
from ..ik.inverse_k import is_valid
import math
import numpy as np
import serial ,time
#this will be the ik function for easy access
J1,J2,J3,J4=20,130,150,60
containers=container_positions.values()
ik_target_pos=list(containers)
print("Containers seen are ")
for i , c in enumerate(ik_target_pos):
    print(f"x={c[0]},y={c[1]:.2f},z={c[2]:.2f}")

port='/dev/ttyUSB0'



def inverse_kinematics(J1,J2,J3,J4,x,y,z):
    z_final=z-J4-J1
    theta1=math.atan2(y,x)
    r=math.hypot(x,y)
    rw=math.hypot(r,z_final)
    if rw>(J2+J3) or rw<abs(J2-J3):
        print("Target out of reach")
        return False
    cos_A3=(J2**2+J3**2-rw**2)/(2*J2*J3)
    cos_A3=max(-1,min(1,cos_A3))
    A3_internal=math.acos(cos_A3)
    theta3_up=math.pi-A3_internal
    theta3_down=A3_internal-math.pi
    beta=math.atan2(z_final,r)
    sin_a=(J3*math.sin(A3_internal))/rw
    sin_a=max(-1,min(1,sin_a))
    alpha=math.asin(sin_a)

    theta2_up=beta-alpha
    theta2_down=beta+alpha

    theta4_down=-1*(theta2_down+theta3_down)
    down=np.array([theta1,theta2_down,theta3_down,theta4_down])

    theta4_up=-1*(theta2_up+theta3_up)
    up=np.array([theta1,theta2_up,theta3_up,theta4_up])
    valid_solutions=[]
    if is_valid(theta2_up,theta3_up):
        valid_solutions.append(up)
    if is_valid(theta2_down,theta3_down):
        valid_solutions.append(down)
    if not valid_solutions:
        print("Violating the joint constraints")
        return False
    return valid_solutions[1] if len(valid_solutions)==2 else valid_solutions[0]#this is just to prioritse the elbow down config
        


def clearence_position(x , y, z):
    clearence_pos=[x-0.5,y-0.5,z-0.5]#keeping dynamic for now will change to static if issue
    print(f"Clearence position is : {clearence_pos[0],clearence_pos[1],clearence_pos[2]}")
    inverse_kinematics(J1,J2,J3,J4,clearence_pos[0],clearence_pos[1],clearence_pos[2])
def motor_movement():
    baud_rate=115200

    try:
        arduino=serial.Serial(port,baud_rate,timeout=1)
        time.sleep(2)
        print("Connected to port ")
    except serial.SerialException as e:
        print(f"Errot :{e}")
        exit()

    def servo_angle(angle):
        try:
            command=str(angle) +'\n'
            arduino.write(command.encode('utf-8'))
            print(f'Angle {angle}')
        except Exception as e:
            print(f"Error :  {e}")
    try :
        while True :
            angle_input=input("Enter desired angle")
            if angle_input.lower()=='q':
                break
            try:
                angle=int(angle_input)
                if 0<=angle<=180:
                    servo_angle(angle)
                else:
                    print('Wrong angle')
            except ValueError:
                print("Invalid input. Please enter an integer.")

    except KeyboardInterrupt:
        print("Program interrupted")

    finally:
        if arduino.is_open:
            arduino.close()
            print("Serial port closed")

def retry():
    pass

#target position movement
target_shelf_pos=[]
def target_shelf():
    for shelf_id,target_positions in detected_aruco_positions.items():
        target_shelf_pos.append(target_positions)
print(target_shelf_pos)


