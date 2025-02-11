import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class Controller(Node):
    """
    A node that commands the velocity and angular speed of the robot
    """
    
    def __init__(self):
        super().__init__('Controller')
        
        self.v = [3,0,-3,0] #speeds in m/s
        self.omega = [0, 90, 180, 270] #speeds in rad/s
        self.i = 0

        self.publisher_ = self.create_publisher(String, 'instructions', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        
        msg = String()
        msg.data = '{}_{}'.format(self.v[self.i], self.omega[self.i])
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing "{0}"'.format(msg.data))
        
        self.i += 1
        if self.i >3: self.i = 0


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
