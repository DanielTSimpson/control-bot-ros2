import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

class HWInterface(Node):
   """
   A node that sends the  motor signals to the Arduino
   """
   def __init__(self):
        super().__init__('hw_interface')
        
        # Initialize the port and Wait half a second to let it initialize
        self.serial_port = serial.Serial(
            port="/dev/ttyACM0", #/dev/ttyTHS1
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        
        rclpy.spin_once(self, timeout_sec=0.1)
        
        self.subscription = self.create_subscription(
            String, 
            'motor_commands', 
            self.listener_callback,
            10)

   def listener_callback(self, msg):
        mtrCmds = msg.data.split("_")
        data = [0]*10
        data[0] = 255
        data[9] = 255 #Set delimiters

        self.get_logger().info(f'Received: M1 {mtrCmds[0]}  M2 {mtrCmds[1]}  M3 {mtrCmds[2]}  M4 {mtrCmds[3]}') 
        
        for i in range(1,9,2):
            speed = int(float(mtrCmds[int((i-1)/2)]))
            if speed < 0:
                data[i] = 1
            else:
                data[i] = 0
            data[i+1] = abs(speed)

        self.serial_port.write(bytearray(data))
        self.get_logger().info(f'Sending {data}')

def main(args=None):
    rclpy.init(args=args)
    hw_interface = HWInterface()
    rclpy.spin(hw_interface)
    hw_interface.destroy_node()
    serial_port.close()
    rclpy.shutdown()


if __name__ == '_main_':
	main()
