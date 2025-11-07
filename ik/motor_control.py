import serial 
import time

port='/dev/ttyUSB0'

baud_rate=115200

try:
    arduino=serial.Serial(port,baud_rate,timeout=1)
    time.sleep(2)
    print("Connected to port ")
except serial.SerialException as e:
    print(f"Errot :{e}")
    exit()

def servo_angle(angle1,angle2 ,angle3 ,angle4):
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
            
