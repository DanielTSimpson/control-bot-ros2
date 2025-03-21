import rclpy

from time import sleep
from rclpy.node import Node
from std_msgs.msg import String

"""
The robot's body reference frame and motor orientation is defined as:
        ^yb
   M1____|___M4
    |    |   |
    |    |   |
    |     ------>xb
    |        |
  M3|________|M2
"""

class Controller(Node):
    """
    A node that commands the velocity and angular speed of the robot
    """
    __v = 65   # speed signal
    __r = 75    # turning radius (mm) in the +x_b direction
    
    def __init__(self):
        super().__init__('Controller')
        
        self.i = 0
        
        self.publisher = self.create_publisher(String, 'instructions', 10)
        self.timer = self.create_timer(3, self.timer_callback)

    def timer_callback(self):
        
        msg = String()
        
        msg.data = '{}_{}'.format(Controller.__v, Controller.__r)
        self.get_logger().info("Publishing {}".format(msg.data))
        self.publisher.publish(msg)
    #    self.i += 1
    #    if self.i > (len(self.inst)-1): self.i = 0


def main(args=None):
    """
    Run a Talker node standalone.

    This function is called directly when using an entrypoint. Entrypoints are configured in
    setup.py. This along with the script installation in setup.cfg allows a talker node to be run
    with the command `ros2 run examples_rclpy_executors talker`.

    :param args: Arguments passed in from the command line.
    """
        
    rclpy.init(args=args)
    hl_ctrl = Controller()
    rclpy.spin(hl_ctrl)
    hl_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Runs a talker node when this script is run directly (not through an entrypoint)
    main()
