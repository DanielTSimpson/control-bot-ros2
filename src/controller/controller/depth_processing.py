import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np



def crop(data, top = 0, right = 0, bottom = 0, left = 0):
    rows = np.size(data, 0)
    cols = np.size(data, 1)
    
    data = data[0:(rows - bottom), 0:(cols - right)]
    data = data[top:, left:]
    
    return data



def show_image(data, window_name="Filtered Image", wait_time = 0, max_width=1600, max_height=1200):
    # Normalize the data for visualization (if necessary)
    normalized_data = cv2.normalize(data, None, 0, 255, cv2.NORM_MINMAX)
    normalized_data = np.uint8(normalized_data)  # Convert to 8-bit image
    normalized_data = cv2.applyColorMap(normalized_data, cv2.COLORMAP_JET)

    # Get original dimensions
    height, width = normalized_data.shape[:2]

    # Calculate scaling factor to fit within max dimensions while preserving aspect ratio
    scale = min(max_width / width, max_height / height)

    # Resize the image to fit the window
    new_width, new_height = int(width * scale), int(height * scale)
    resized_image = cv2.resize(normalized_data, (new_width, new_height))

    # Create resizable window and display the image
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    
    cv2.resizeWindow(window_name, new_width, new_height)
    cv2.imshow(window_name, resized_image)

    # Wait for a key press and then close the window
    cv2.waitKey(wait_time)
    cv2.destroyAllWindows()



def filter(data, node):    
    data_rows = np.size(data, 0)  # Number of Rows
    data_cols = np.size(data, 1)  # Number of Columns
    K_rows = data_rows // 1  #Filter window height. Must be factor of data_rows
    K_cols = data_cols // data_cols  #Filter window width. Must be factor of data_cols
    
    # Filter Matrix (for averaging)
    K = np.ones((K_rows, K_cols)) / (K_rows * K_cols)

    # Correctly initialize the resultant matrix with zeros
    R = np.zeros((data_rows // K_rows, data_cols // K_cols))

    for i in range(0, data_rows, K_rows):
        for j in range(0, data_cols, K_cols):
            sub_arr = data[i:(i + K_rows), j:(j + K_cols)]

            # Perform element-wise multiplication and sum the result for averaging
            R[i // K_rows, j // K_cols] = np.sum(sub_arr * K)

    show_image(R, "Image", 500)
    loss = int(np.argmax(R)) - data_cols // 2
    return Int32(data = loss)



class DepthProcessor(Node):
    def __init__(self):
        super().__init__('DepthProcessor')
        
        self.loss = Int32()
        self.loss.data = 0

        self.subscription = self.create_subscription(
            Image,
            'realsense2_camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Int32, 'loss_function', 10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)



    def listener_callback(self, msg):
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        data = np.asarray(depth_image, dtype="int32")
        data = crop(data, 0, 0, 180, 24)
        self.loss = filter(data, self)



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

