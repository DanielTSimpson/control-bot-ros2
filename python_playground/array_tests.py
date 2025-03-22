# Made this for iterating and testing the edge cases of my crop function

import numpy as np
import cv2

def crop(data, top, right, bottom, left):
    rows = np.size(data, 0)
    cols = np.size(data, 1)
    
    data = data[0:(rows - bottom), 0:(cols - right)]
    data = data[top:, left:]
    
    return data


def show_image(data, window_name="Filtered Image", max_width=1600, max_height=1200):
    # Normalize the data for visualization (if necessary)
    normalized_data = cv2.normalize(data, None, 0, 255, cv2.NORM_MINMAX)
    normalized_data = np.uint8(normalized_data)  # Convert to 8-bit image
    normalized_data = cv2.applyColorMap(normalized_data, cv2.COLORMAP_JET)

    # Get original dimensions
    height, width = normalized_data.shape[:2]

    # Calculate scaling factor to fit within max dimensions while preserving aspect ratio
    scale = min(max_width / width, max_height / height)

    # Resize the image to fit the window
    new_width, new_height = int(width * scale), int(height * scale * 1.5)
    resized_image = cv2.resize(normalized_data, (new_width, new_height))

    # Create resizable window and display the image
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    cv2.resizeWindow(window_name, new_width, new_height)
    cv2.imshow(window_name, resized_image)

    # Wait for a key press and then close the window
    cv2.waitKey(0)
    cv2.destroyAllWindows()





def main(args = None):
    K_rows = 24  #Filter window height. Must be factor of data_rows
    K_cols = 64  #Filter window width. Must be factor of data_cols
    # Filter Matrix (for averaging)
    K = np.ones((K_rows, K_cols)) / (K_rows * K_cols)

    K[0,:] = K[0,:]*0
    K[:,0] = K[:,0]*0
    K[K_rows-1,:] = K[K_rows-1,:]*0
    K[:,K_cols-1] = K[:,K_cols-1]*0
    
    show_image(K)
    K = crop(K, 0, 0, 0, 1)
    show_image(K)

main()
