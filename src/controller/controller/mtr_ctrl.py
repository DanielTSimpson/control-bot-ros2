import rclpy
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


class MotorCtrl(Node):
    """
    A node that converts desired robot velocity and attitudes into motor signals
    """
    __w = 110 #Width between wheel placements (mm)

    def __init__(self):
        self.mtr1 = 0.0
        self.mtr2 = 0.0
        self.mtr3 = 0.0
        self.mtr4 = 0.0

        super().__init__('mtr_ctrl')

        self.subscription = self.create_subscription(
            String, 
            'instructions', 
            self.listener_callback,
            10)
        self.subscription

        self.publisher = self.create_publisher(String, 'motor_commands', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def listener_callback(self, msg):
        v, r = msg.data.split('_')
        v = float(v)
        r = float(r)

        v1 = v + MotorCtrl.__w*v/2/r
        v2 = v - MotorCtrl.__w*v/2/r

        # Keep all motor commands between 0 - 30
        self.mtr1 = v1
        self.mtr2 = v2
        self.mtr3 = v1
        self.mtr4 = v2
        
        self.get_logger().info('I heard: {} signal at {} mm'.format(v, r)) 
    
    def timer_callback(self):
        msg = String()
        msg.data = "{}_{}_{}_{}".format(self.mtr1, self.mtr2, self.mtr3, self.mtr4)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: {}'.format(msg.data))

def main(args=None):
	
    rclpy.init(args=args)
    
    mtr_ctrl = MotorCtrl()
    
    rclpy.spin(mtr_ctrl)
    
    mtr_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '_main_':
	main()
