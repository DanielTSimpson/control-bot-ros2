import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


def show_image(data, window_name="Filtered Image", max_width=800, max_height=600):
    # Normalize the data for visualization (0-255 for colormap application)
    normalized_data = cv2.normalize(data, None, 0, 255, cv2.NORM_MINMAX)
    normalized_data = np.uint8(normalized_data)

    # Apply a colormap (e.g., COLORMAP_JET for a rainbow effect)
    colored_image = cv2.applyColorMap(normalized_data, cv2.COLORMAP_JET)

    # Get original dimensions
    height, width = colored_image.shape[:2]

    # Calculate scaling factor to fit within max dimensions while preserving aspect ratio
    scale = min(max_width / width, max_height / height)

    # Resize the image to fit the window
    new_width, new_height = int(width * scale), int(height * scale)
    resized_image = cv2.resize(colored_image, (new_width, new_height), interpolation=cv2.INTER_AREA)

    # Create resizable window and display the image
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, new_width, new_height)
    cv2.imshow(window_name, resized_image)

    # Wait for a key press and then close the window
    cv2.waitKey(1000)
    cv2.destroyAllWindows()


def filter(data, node):
    K_s = 8  # Side length of filter (must be divisible by 480, 640)
    data_rows = np.size(data, 0)  # Number of Rows
    data_columns = np.size(data, 1)  # Number of Columns

    # Filter Matrix (for averaging)
    K = np.ones((K_s, K_s)) / (K_s * K_s)

    # Correctly initialize the resultant matrix with zeros
    R = np.zeros((data_rows // K_s, data_columns // K_s))

    for i in range(0, data_rows, K_s):
        for j in range(0, data_columns, K_s):
            sub_arr = data[i:(i + K_s), j:(j + K_s)]

            # Perform element-wise multiplication and sum the result for averaging
            R[i // K_s, j // K_s] = np.sum(sub_arr * K)

    show_image(R)
    return Float32(data=R[1,1])



class DepthProcessor(Node):
    def __init__(self):
        super().__init__('DepthProcessor')
        
        self.loss = Float32()
        self.loss.data = 0.0

        self.subscription = self.create_subscription(
            Image,
            'realsense2_camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Float32, 'loss_function', 10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        data = np.asarray(depth_image, dtype="int32")
        self.loss = filter(depth_image, self)

    def timer_callback(self):
        if self.loss:
            self.publisher.publish(self.loss)
            self.get_logger().info(f"Published loss: {self.loss.data}")


def main(args=None):
    rclpy.init(args=args)
    depth_processor = DepthProcessor()
    rclpy.spin(depth_processor)
    depth_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

