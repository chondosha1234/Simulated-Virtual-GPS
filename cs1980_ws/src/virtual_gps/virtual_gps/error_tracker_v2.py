import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

import json

# loading error data
file_path1 = "./measurements/error_data1.json"
with open(file_path1, 'r') as json_file:
    error_data1 = json.load(json_file)

# loading error data
file_path2 = "./measurements/error_data2.json"
with open(file_path2, 'r') as json_file:
    error_data2 = json.load(json_file)

# loading error data
file_path3 = "./measurements/error_data3.json"
with open(file_path3, 'r') as json_file:
    error_data3 = json.load(json_file)

count = min([len(error_data1), len(error_data2), len(error_data3)])

# data that will be plotted
time = np.linspace(0, 1, count)  # Time from 0 to 10, with 100 points

error_vals1 = [val["error"] for val in error_data1[:count]]
error_vals2 = [val["error"] for val in error_data2[:count]]
error_vals3 = [val["error"] for val in error_data3[:count]]


plt.figure(figsize=(10,6))

# Line plot of value vs. time
plt.plot(time, error_vals1, color='b', label="4 Drones")  
plt.plot(time, error_vals2, color='g', label="2 Drones")
plt.plot(time, error_vals3, color='r', label="1 Drone")

plt.legend()

# Adding labels and title
plt.xlabel('Time')
plt.ylabel('Error')
plt.title('Error over Time')

# modifying x and y ticks
plt.xticks([])
plt.gca().yaxis.set_major_locator(MaxNLocator(integer=True, prune='lower'))  # Prunes the lower end
plt.gca().yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x:.4f}'))  # Format the ticks to 4 decimal places

# Show the plot
plt.show()
