import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from .display_helper import Display
from scipy.signal import find_peaks, peak_widths

def crop(data, top = 0, right = 0, bottom = 0, left = 0):
    rows = np.size(data, 0)
    cols = np.size(data, 1)
    
    data = data[0:(rows - bottom), 0:(cols - right)]
    data = data[top:, left:]
    
    return data


def filter_img(data, node):   
    data_rows = np.size(data, 0)  
    data_cols = np.size(data, 1)  
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
    
    return R
    

def filter_depth(data, node): 
    data_rows = np.size(data, 0)  
    data_cols = np.size(data, 1)  
    K_rows = data_rows // 1  #Filter window height. Must be factor of data_rows
    K_cols = data_cols // data_cols * 8  #Filter window width. Must be factor of data_cols
    
    # Filter Matrix (for averaging)
    K = np.ones((K_rows, K_cols)) / (K_rows * K_cols)

    # Correctly initialize the resultant matrix with zeros
    R = np.zeros((data_rows // K_rows, data_cols // K_cols))

    # Reduce the camera's range below the threshold

    data = np.where(data > 750, 0, data)
    data = np.where(data < 10, 0, data)

    for i in range(0, data_rows, K_rows):
        for j in range(0, data_cols, K_cols):
            sub_arr = data[i:(i + K_rows), j:(j + K_cols)]

            # Perform element-wise multiplication and sum the result for averaging
            R[i // K_rows, j // K_cols] = np.sum(sub_arr * K)

    peaks, _ = find_peaks(R[0], prominence = 20, height=50)
    widths = peak_widths(R[0], peaks, rel_height=0.75)
    objects = np.unique((widths[2] + widths[3])/2).astype(int)
    
    #node.display.show(data, "Depth Image")
    node.display.show(R[0], "Depth Plot")
    #node.display.show(peaks, "Peaks")
    
    print(objects)
    if len(objects) != 0:
        loss = (objects[0] - len(R[0])/2) / (len(R[0])/2) #Loss as %-diff from middle
    else:
        loss = 0.0
    return Float32(data = loss)


class DepthProcessor(Node):

    def __init__(self):
        super().__init__('DepthProcessor')
        
        self.loss = Float32()
        self.loss.data = 0.0
        
        timer_period = 0.05
        self.display = Display(int(timer_period*1000))

        self.sub1 = self.create_subscription(
            Image,
            'realsense2_camera/depth/image_rect_raw',
            self.callback1,
            10)
        
        #self.sub2 = self.create_subscription(
        #    Image,
        #    'realsense2_camera/color/image_raw',
        #    self.callback2,
        #    10)

        self.publisher = self.create_publisher(Float32, 'loss_function', 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def callback1(self, msg):
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        data = np.asarray(depth_image, dtype="int32")
        data = crop(data, 0, 0, 196, 48)        
        self.loss = filter_depth(data, self)

    def callback2(self, msg):
        bridge = CvBridge()
        color_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        data = np.asarray(color_img, dtype="uint8")
        data = crop(data, 0, 0, 0, 48) 
        color_img = filter_img(data, self)
        #self.display.show(color_img, "Color Image")

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
