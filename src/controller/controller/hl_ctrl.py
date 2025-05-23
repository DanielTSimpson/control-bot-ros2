import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32

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
    __Kp = 1    # Proportion constant
    __Ki = 0    # Integral constant
    __Kd = 0    # Derivative constant

    __M1 = 0
    __M2 = 0
    __M3 = 0
    __M4 = 0

    __minLoss = 0.06

    def __init__(self):
        super().__init__('Controller')
        
        self.i = 0
        self.timer_period = 1/16 

        self.sub1 = self.create_subscription(
                Float32,
                'loss_function',
                self.callback1,
                10)

        self.publisher = self.create_publisher(String, 'motor_commands', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def callback1(self, msg):
        loss = msg.data
        
        if abs(loss) < Controller.__minLoss:
            result = 0
        else:
            result = int(100 + 155*abs(loss))

        if loss > 0:
            Controller.__M1 = result
            Controller.__M3 = result
            Controller.__M2 = -result
            Controller.__M4 = -result
        else:
            Controller.__M1 = -result
            Controller.__M3 = -result
            Controller.__M2 = result
            Controller.__M4 = result
        
    def timer_callback(self): 
        msg = String()
        msg.data = '{}_{}_{}_{}'.format(Controller.__M1, Controller.__M2, Controller.__M3, Controller.__M4)
        self.get_logger().info("Publishing {}".format(msg.data))
        self.publisher.publish(msg)

    def on_shutdown(self):
        self.get_logger().info("Node is shutting down. Sending failsafe command.")
        msg = String()
        msg.data = "0_0_0_0"
        self.publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)

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
    try:
        rclpy.spin(hl_ctrl)
    except KeyboardInterrupt:
        pass
    finally:
        hl_ctrl.on_shutdown()
        hl_ctrl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Runs a talker node when this script is run directly (not through an entrypoint)
    main()
