from scipy.signal import find_peaks, peak_widths
from matplotlib import pyplot as plt
import numpy as np

sub_height = 4
sub_width = 8
fig, axs = plt.subplots(sub_height, sub_width)

for j in range(sub_height):
    for k in range(sub_width):
        # Example data
        data = np.random.randint(0,5,(1,70))[0]
        peaks = np.random.randint(1,70,(1,3))[0]

        # Manufacture some flat peaks
        for i in peaks:
            data[i-3:i+3] = data[i-3:i+3] + 100

        # Show data
        #print(data)
        #print(peaks)

        # Find peaks
        peaks, _ = find_peaks(data, prominence=95)#, width=5)#, distance=3)#, prominence=0.3)
        widths_half = peak_widths(data, peaks, rel_height=0.75)
        peak_locations = np.unique((widths_half[2] + widths_half[3]) / 2).astype(int)
        
        print("Widths Halves: " + str(widths_half))
        print("Peak Locations: " + str(peak_locations))
        #print("Peaks are at indices:", peaks)

        # Plot the data
        axs[j][k].plot(data, label="Data")
        axs[j][k].plot(peak_locations, data[peak_locations], "x", label="Peaks")
        axs[j][k].hlines(*widths_half[1:], color="C2")
#plt.hlines(*widths_full[1:], color="C3")
plt.pause(0)
plt.close()