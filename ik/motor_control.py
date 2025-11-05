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
        
