from check import run_object_detection
from inverse_k import is_valid
import math
import numpy as np
import serial ,time
import threading
global clearence_pos
J1,J2,J3,J4=20,130,150,60
container_pos,aruco_pos=run_object_detection()
containers=container_pos.values()
ik_target_pos=list(containers)
target_shelf_pos=[]

for i in aruco_pos.values():
     target_shelf_pos.append(i)

print("Containers seen are ")
for i , c in enumerate(ik_target_pos):
    print(f"x={c[0]},y={c[1]:.2f},z={c[2]:.2f}")

port='/dev/ttyUSB0'
baud_rate=115200
try:
        arduino=serial.Serial(port,baud_rate,timeout=1)
        time.sleep(2)
        print("Connected to port ")
except serial.SerialException as e:
        print(f"Errot :{e}")
        exit()

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
        
def gripper_action_open():
    try:
         command_open=str(90)+'\n'
        #  command_close=str(90)+'\n'
         arduino.write(command_open.encode('utf-8'))
         print(f"Gripper opened angle put as {command_open}")
    except Exception as e:
         print(f"Error : {e}")
    except KeyboardInterrupt:
         print("Keyboard stopped")
    finally:
         if arduino.is_open:
              arduino.close()
              print("Serial port closed")

def gripper_action_close():
    try:
         angle=-1*90
         command_close=str(angle)+'\n'
         arduino.write(command_close.encode('utf-8'))
         print(f"Gripper closed at angles :{command_close}")
    except Exception as e:
         print(f"Error : {e}")
    except KeyboardInterrupt:
         print("Keyboard inturrupted")
    finally:
         if arduino.is_open:
              arduino.close()
              print("Serial port closed")
            
         
    


def clearence_position(x , y, z):
    global clearence_pos
    clearence_pos=[x-0.5,y-0.5,z-0.5]#keeping dynamic for now will change to static if issue
    print(f"Clearence position is : {clearence_pos[0],clearence_pos[1],clearence_pos[2]}")
    inverse_kinematics(J1,J2,J3,J4,clearence_pos[0],clearence_pos[1],clearence_pos[2])
def motor_movement(angle1 , angle2 , angle3 , angle4):

    
    try:
            command1=str(angle1) +'\n'
            arduino.write(command1.encode('utf-8'))
            command2=str(angle2)+'\n'
            arduino.write(command2.encode('utf-8'))
            command3=str(angle3)+'\n'
            arduino.write(command3.encode('utf-8'))
            command4=str(angle4)+'\n'
            arduino.write(command4.encode('utf-8'))
            print(f"All the angles sent {angle1,angle2,angle3,angle4}")

    except Exception as e:
            print(f"Error :  {e}")
    # try :
    #     while True :
    #         angle_input=input("Enter desired angle")
    #         if angle_input.lower()=='q':
    #             break
    #         try:
    #             angle=int(angle_input)
    #             if 0<=angle<=180:
    #                 servo_angle(angle)
    #             else:
    #                 print('Wrong angle')
    #         except ValueError:
    #             print("Invalid input. Please enter an integer.")

    except KeyboardInterrupt:
            print("Program interrupted")

    finally:
            if arduino.is_open:
                arduino.close()
                print("Serial port closed")

def retry(current_container_X,current_container_Y,current_container_Z):
    check_container_pos , check_aruco_pos=run_object_detection()
    for i,c in check_container_pos.items():
        if c[0]==current_container_X+0.5 and c[1]==current_container_Y+0.5 and c[3]==current_container_Z+0.5:
            return True
        print(f"Container {i} has not been picked up")
    return False
         

#target position movement
target_shelf_pos=[]
def target_shelf():
    for shelf_id,target_positions in aruco_pos.items():
        target_shelf_pos.append(target_positions)
print(target_shelf_pos)


print("Moving to the clearence position")
clearence_x_angle , clearence_y_angle2,clearence_z_angle3,clearence_z_angle4=clearence_position()
motor_movement(clearence_x_angle,clearence_y_angle2,clearence_z_angle3,clearence_z_angle4)

print(f"Reached the position : {clearence_pos}")
i=1#testing for now 
for container in containers:
    print(f"Moving to approch position of the {i+1} container")
    approach_pos=[container[0]-0.05,container[1]-0.08,container[2]-0.08]
    angle1 , angle2 , angle3,angle4= inverse_kinematics(approach_pos[0],approach_pos[1],approach_pos[2])
    print(f"Angles formed are : {angle1 , angle2 ,angle3 ,angle4}")
    print("Executing motion")
    motor_movement(angle1,angle2,angle3,angle4)
    print("Motion complete gripper should be at approach position")
    print("Moving to the final position of the container")
    final_position_ik=[container[0],container[1],container[2]-J4]
    print("The final position coordinates are : {final_position_ik}")
    print("Moving to the final position for container")
    theta1,theta2,theta3,theta4=inverse_kinematics(final_position_ik[0],final_position_ik[1],final_position_ik[2])
    print(f"Angles formed : {theta1,theta2,theta3,theta4}")
    print("Moving to position")
    motor_movement(theta1,theta2,theta3,theta4)
    print(f"Reached the final position")
    print(f"Holding the container ")
    gripper_action_close()
    print("Closed the gripper and holding the container")
    print(f"Returning to clearence position : {clearence_pos}")
    t1 , t2 , t3 , t4=clearence_position()
    print(f"The angles formed are {t1 ,t2,t3,t4}")
    motor_movement(t1,t2,t3,t4)
    print("The arm has reached the clearence position")
    is_picked_up=retry(container[0],container[1],container)
    if not is_picked_up :
        print(f"Log : picked up the container at position : {container[0],container[1],container[2]}")
        continue
    else:
        print(f"Moving to approch position of the {i+1} container")
        approach_pos=[container[0]-0.05,container[1]-0.08,container[2]-0.08]
        angle1 , angle2 , angle3,angle4= inverse_kinematics(approach_pos[0],approach_pos[1],approach_pos[2])
        print(f"Angles formed are : {angle1 , angle2 ,angle3 ,angle4}")
        print("Executing motion")
        motor_movement(angle1,angle2,angle3,angle4)
        print("Motion complete gripper should be at approach position")
        print("Moving to the final position of the container")
        final_position_ik=[container[0],container[1],container[2]-J4]
        print("The final position coordinates are : {final_position_ik}")
        print("Moving to the final position for container")
        theta1,theta2,theta3,theta4=inverse_kinematics(final_position_ik[0],final_position_ik[1],final_position_ik[2])
        print(f"Angles formed : {theta1,theta2,theta3,theta4}")
        print("Moving to position")
        motor_movement(theta1,theta2,theta3,theta4)
        print(f"Reached the final position")
        print(f"Holding the container ")
        gripper_action_close()
        print("Closed the gripper and holding the container")
        print(f"Returning to clearence position : {clearence_pos}")
        t1 , t2 , t3 , t4=clearence_position()
        print(f"The angles formed are {t1 ,t2,t3,t4}")
        motor_movement(t1,t2,t3,t4)
        print("The arm has reached the clearence position")
        is_picked_up=retry()
        if is_picked_up :
            print(f"Log : picked up the container at position : {container[0],container[1],container[2]}")
            continue
        else:
            print(f"Moving to approch position of the {i+1} container")
            approach_pos=[container[0]-0.05,container[1]-0.08,container[2]-0.08]
            angle1 , angle2 , angle3,angle4= inverse_kinematics(approach_pos[0],approach_pos[1],approach_pos[2])
            print(f"Angles formed are : {angle1 , angle2 ,angle3 ,angle4}")
            print("Executing motion")
            motor_movement(angle1,angle2,angle3,angle4)
            print("Motion complete gripper should be at approach position")
            print("Moving to the final position of the container")
            final_position_ik=[container[0],container[1],container[2]-J4]
            print(f"The final position coordinates are : {final_position_ik}")
            print("Moving to the final position for container")
            theta1,theta2,theta3,theta4=inverse_kinematics(final_position_ik[0],final_position_ik[1],final_position_ik[2])
            print(f"Angles formed : {theta1,theta2,theta3,theta4}")
            print("Moving to position")
            motor_movement(theta1,theta2,theta3,theta4)
            print(f"Reached the final position")
            print(f"Holding the container ")
            gripper_action_close()
            print("Closed the gripper and holding the container")
            print(f"Returning to clearence position : {clearence_pos}")
            t1 , t2 , t3 , t4=clearence_position()
            print(f"The angles formed are {t1 ,t2,t3,t4}")
            motor_movement(t1,t2,t3,t4)
            print("The arm has reached the clearence position")
            is_picked_up=retry()
            if is_picked_up :
                print(f"Log : picked up the container at position : {container[0],container[1],container[2]}")
                continue
            else:
                print("Log:Pick up not successfull")
    print(f"Moving to target shelf positions : {target_shelf_pos}")
    target_theta1,target_theta2,target_theta3,target_theta4=inverse_kinematics(target_shelf_pos[0],target_shelf_pos[1],target_shelf_pos[2])
    print(f"Angles to move to target shelf clearence is : {target_theta1,target_theta2,target_theta3,target_theta4}")
    motor_movement(target_theta1,target_theta2,target_theta3,target_theta4)
    print("Reached the target shelf clearence position")
    target_shelf_approach_pos=[target_shelf_pos[0],target_shelf_pos[1],target_shelf_pos[2]]
    print(f"Movig to the target position approach :{target_shelf_approach_pos}")
    tt1,tt2,tt3,tt4=inverse_kinematics(target_shelf_approach_pos[0],target_shelf_approach_pos[1],target_shelf_approach_pos[2])
    print(f"angles formed : {tt1,tt2,tt3,tt4}")
    motor_movement(tt1,tt2,tt3,tt4)
    print(f"Reached the approach target pos")
    print(f"moving to final target final position {target_shelf_pos}")
    final_angle1,final_angle2,final_angle3,final_angle4=inverse_kinematics(target_shelf_pos[0],target_shelf_pos[1],target_shelf_pos[2])
    print(f"Angle values : {final_angle1,final_angle2,final_angle3,final_angle4}")
    motor_movement(final_angle1,final_angle2,final_angle3,final_angle4)
    print(f"Reached the final target position")
    gripper_action_open()
    print("Released the container now moving back to clearence position")
    c1,c2,c3,c4=inverse_kinematics(target_shelf_pos[0],target_shelf_pos[1],target_shelf_pos[2])
    print(f"The angles formed are : {c1,c2,c3,c4}")
    motor_movement(c1,c2,c3,c4)





    
    
        
         
    


   

