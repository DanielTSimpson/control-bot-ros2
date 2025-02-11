#!/usr/bin/python3
import time
import serial

print("UART Demonstration Program")
print("NVIDIA Jetson Nano Developer Kit")

serial_port = serial.Serial(
    port="/dev/ttyACM0", #/dev/ttyTHS1
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

# Wait a second to let the port initialize
time.sleep(1)

try:
    while True:
        
        # invalid bytes: 0000000(0x00) and 11111111(0xFF)
       for i in range(4): 
        # print(i)
        direction = str(bin(bool(i%2))[2:]) 
        motor_id = format(i,'02b') 
        speed = format(f"{5:b}",'05')
        
        # print("Direction: " + direction  + " Motor ID: " + motor_id + " Speed: " + speed) 
        binary_string = direction + motor_id  + speed 
        serial_port.write(binary_string.encode())
        
        # print("Sent: " + binary_string)
        time.sleep(1)      

except KeyboardInterrupt:
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    serial_port.close()
    pass

