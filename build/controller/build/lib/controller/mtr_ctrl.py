import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MotorCtrl(Node):
    """
    A node that converts desired robot velocity and attitudes into motor signals
    """
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
        v, theta = msg.data.split('_')
        v = float(v)
        omega = float(theta)

        # Keep all motor commands between 0 - 30
        self.mtr1 = v
        self.mtr2 = v
        self.mtr3 = v
        self.mtr4 = v

        self.get_logger().info('I heard: {} m/s at {} deg'.format(v, theta)) 
    
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
