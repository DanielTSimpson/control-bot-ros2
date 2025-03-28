import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
plt.ion() # Enable Interactive Mode



def plot_data(data):
    """
    Plot the data (a 1D array) using matplotlib
    """
    plt.clf()  # Clear previous plot
    
    plt.plot(data, linewidth = 7)
    plt.plot(np.argmin(data), np.min(data), color='green', marker='o', linestyle='dashed',
             linewidth=2, markersize=12) 
    plt.ylim(-1000, 5000)
    plt.ylabel("Averaged Vertical Depth Value")
    plt.xlabel("Horizontal Pixels")
    
    plt.pause(0.1)  # Pause to allow for rendering



def crop(data, top = 0, right = 0, bottom = 0, left = 0):
    rows = np.size(data, 0)
    cols = np.size(data, 1)
    
    data = data[0:(rows - bottom), 0:(cols - right)]
    data = data[top:, left:]
    
    return data



def show_img(data, node, loss = 0, window_name="Filtered Image", wait_time = 0, max_width=1200, max_height=1800, colorize = True):

    # Normalize the data for visualization (if necessary)
    normalized_data = np.uint8(data)  # Convert to 8-bit image
    if colorize:
        normalized_data = cv2.normalize(normalized_data, None, 0, 255, cv2.NORM_MINMAX)
        normalized_data = cv2.applyColorMap(normalized_data, cv2.COLORMAP_JET)
    
    # Get original dimensions
    height, width = normalized_data.shape[:2]
    
    #Visualize loss with a horizontal line
    if loss:
        node.get_logger().info(str(loss))
        cv2.line(normalized_data, (width//2, height//2), (width//2 + loss, height//2), (255, 0, 0), 1)

        # Calculate scaling factor to fit within max dimensions while preserving aspect ratio
        scale = min(max_width / width, max_height / height)

        # Resize the image to fit the window
        new_width, new_height, new_loss = int(width * scale), int(height * scale), int(loss * scale)
        resized_image = cv2.resize(normalized_data, (new_width, new_height))

        # Create resizable window and display the image
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        resized_image = cv2.cvtColor(resized_image, cv2.COLOR_RGB2BGR) # Convert RGB to BGR
        cv2.resizeWindow(window_name, new_width, new_height)
        cv2.imshow(window_name, resized_image)

        # Wait for a key press and then close the window
        cv2.waitKey(wait_time)
        cv2.destroyAllWindows()



def filter_img(data, node):   
    data_rows = np.size(data, 0)  # Number of Rows
    data_cols = np.size(data, 1)  # Number of Columns
    K_rows = data_rows // data_rows * 8  #Filter window height. Must be factor of data_rows
    K_cols = data_cols // data_cols * 8  #Filter window width. Must be factor of data_cols
    
    # Filter Matrix (for averaging)
    K = np.ones((K_rows, K_cols)) / (K_rows * K_cols)

    # Initialize the resultant matrix with zeros
    R = np.zeros((data_rows // K_rows, data_cols // K_cols, 3))

    for i in range(0, 3):
        for j in range(0, data_rows, K_rows):
            for k in range(0, data_cols, K_cols):
                sub_arr = data[j:(j + K_rows), k:(k + K_cols), i]

                # Perform element-wise multiplication and sum the result for averaging
                R[j // K_rows, k // K_cols, i] = np.sum(sub_arr * K)
    
    return R, K_cols
    


def filter_depth(data, node):    
    data_rows = np.size(data, 0)  # Number of Rows
    data_cols = np.size(data, 1)  # Number of Columns
    K_rows = data_rows // 1  #Filter window height. Must be factor of data_rows
    K_cols = data_cols // data_cols * 8  #Filter window width. Must be factor of data_cols
    
    # Filter Matrix (for averaging)
    K = np.ones((K_rows, K_cols)) / (K_rows * K_cols)

    # Correctly initialize the resultant matrix with zeros
    R = np.zeros((data_rows // K_rows, data_cols // K_cols))

    for i in range(0, data_rows, K_rows):
        for j in range(0, data_cols, K_cols):
            sub_arr = data[i:(i + K_rows), j:(j + K_cols)]

            # Perform element-wise multiplication and sum the result for averaging
            R[i // K_rows, j // K_cols] = np.sum(sub_arr * K)
   
    loss = int(np.argmin(R) - np.size(R, 1) / 2) * K_cols
    #node.get_logger().info("Plot Loss: " + str(int(np.argmin(R)) - (np.size(R, 1)) / 2) +
    #        "\tTrue Loss: " + str(loss)
    #        )
    
    #Plot data array
    plot_data(R[0])
    
    
    return Int32(data = loss)



class DepthProcessor(Node):

    def __init__(self):
        super().__init__('DepthProcessor')
        
        self.loss = Int32()
        self.loss.data = 0

        self.sub1 = self.create_subscription(
            Image,
            'realsense2_camera/depth/image_rect_raw',
            self.callback1,
            10)
        
        self.sub2 = self.create_subscription(
            Image,
            'realsense2_camera/color/image_raw',
            self.callback2,
            10)

        self.publisher = self.create_publisher(Int32, 'loss_function', 10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def callback1(self, msg):
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        data = np.asarray(depth_image, dtype="int32")
        data = crop(data, 0, 0, 480//2, 24)
        self.loss = filter_depth(data, self)

    def callback2(self, msg):
        bridge = CvBridge()
        color_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        data = np.asarray(color_img, dtype="uint8")
        color_img, filter_scale = filter_img(data, self)
        self.get_logger().info("Filter Scale is: " + str(filter_scale))
        if self.loss:
            show_img(color_img, self, wait_time = 0, loss = self.loss.data // filter_scale, colorize = False)

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

