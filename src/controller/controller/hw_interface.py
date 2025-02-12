import rclpy
import time
import serial

from rclpy.node import Node
from std_msgs.msg import String
class HWInterface(Node):

    mtr_cmds = [0]*4

    """
    A node that sends the  motor signals to the Arduino
    """
    def __init__(self):

        # Initialize the port and Wait half a second to let it initialize
        self.serial_port = serial.Serial(
            port="/dev/ttyACM0", #/dev/ttyTHS1
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        time.sleep(0.5)
        
        super().__init__('mtr_ctrl')

        self.subscription = self.create_subscription(
            String, 
            'motor_commands', 
            self.listener_callback,
            10)
        self.subscription
     

    def listener_callback(self, msg):
        mtr_cmds = msg.data.split("_")
        self.get_logger().info('Received: M1 {}  M2 {}  M3 {}  M4 {}'.format(mtr_cmds[0], mtr_cmds[1], mtr_cmds[2], mtr_cmds[3])) 
         
        for i in range(4):
            self.serial_port.write(int(float(mtr_cmds[i])).to_bytes(1, 'big'))
            self.get_logger().info('Sending {} for Motor {}'.format(mtr_cmds[i], i+1))
            time.sleep(0.1)
            """direction = int(0)
            
            if int(float(mtr_cmds[i])) > 0: direction = int(1)
            
            direction = str(bin(bool(direction%2))[2:]) 
            motor_id = format(i,'02b') 
            speed = format(f"{int(float(mtr_cmds[i])):b}",'05')
            # print("Direction: " + direction  + " Motor ID: " + motor_id + " Speed: " + speed) 
           
            # invalid bytes: 0000000(0x00) and 11111111(0xFF)
            binary_string = direction + motor_id  + speed 
            binary_string = "255_100_123"
            self.serial_port.write(binary_string.encode())
            #test = int(12)
            #self.serial_port.write(test.to_bytes(2,'big'))
            self.get_logger().info('Sending {} for Motor {}'.format(speed, motor_id))
            time.sleep(1)""" 

def main(args=None):
    rclpy.init(args=args)
    hw_interface = HWInterface()
    rclpy.spin(hw_interface)
    hw_interface.destroy_node()
    serial_port.close()
    rclpy.shutdown()


if __name__ == '_main_':
	main()
