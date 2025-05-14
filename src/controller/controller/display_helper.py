
try:
    import matplotlib.pyplot as plt
    import cv2
    import numpy as np
    plt.ion() #Enable interactive mode

except ImportError as e:
    print(f"Error importing library {e.name}")

class Display(object):
    """ """

    def __init__(self, wait_time = 1000, loss = 0.0): #wait_time in milliseconds
        self.wait_time = wait_time
        self.loss = loss
        self.max_width = 1200
        self.max_height = 800
        self.window_name = "Display"

    def show(self, data, window_name = None):
        self.window_name = window_name if window_name else self.window_name
        if len(np.shape(data)) == 1:
            self.show_plot(data)
        elif len(np.shape(data)) == 2 or len(np.shape(data)) == 3:
            self.show_img(data)
        else:
            print("Incompatable data array dimensions %i" % len(data))


    def show_plot(self, data, points = None, widths = None):
        plt.clf()

        plt.plot(data, linewidth = 7)
        if points: plt.plot(points, data[points], 'x')
        if widths: plt.plot(*widths[1:], color="C2")
        
        plt.ylim(-200,500)
        plt.ylabel("Vertical depth values")
        plt.xlabel("Horizontal Pixels")
        plt.title(self.window_name)

        plt.pause(0.05)


    def show_img(self, data):
        normed_data = cv2.normalize(data, None, 0, 255, cv2.NORM_MINMAX)
        normed_data = np.uint8(normed_data)
        if len(np.shape(data)) == 2: 
            normed_data = cv2.applyColorMap(normed_data, cv2.COLORMAP_JET)
#            print(f"Max: {np.max(data)}\tMin: {np.min(data)}\tMed: {np.median(data)}") 
        elif len(np.shape(data)) == 3: normed_data = cv2.cvtColor(normed_data, cv2.COLOR_RGB2BGR)

        height, width = normed_data.shape[:2]
        scale = min(self.max_width / width, self.max_height / height)
        new_width, new_height = int(width * scale), int(height * scale)
        resized_img = cv2.resize(normed_data, (new_width, new_height))
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow(self.window_name, new_width, new_height)
        cv2.imshow(self.window_name, resized_img)

        cv2.waitKey(self.wait_time)
