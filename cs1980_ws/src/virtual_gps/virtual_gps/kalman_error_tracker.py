import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

import json

# loading error data
file_path = "./measurements/kalman_error_data.json"
with open(file_path, 'r') as json_file:
    error_data = json.load(json_file)

count = len(error_data)

# data that will be plotted
time = np.linspace(0, 1, count)  # Time from 0 to 10, with 100 points
error_vals = [val["error"] for val in error_data]

plt.figure(figsize=(10,6))

# Create the plot
plt.plot(time, error_vals, color='b')  # Line plot of value vs. time

# Adding labels and title
plt.xlabel('Time')
plt.ylabel('Error')
plt.title('Kalman Error over Time')

# modifying x and y ticks
plt.xticks([])
plt.gca().yaxis.set_major_locator(MaxNLocator(integer=True, prune='lower'))  # Prunes the lower end
plt.gca().yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x:.4f}'))  # Format the ticks to 4 decimal places

plt.ylim(0.0, 20.0)

# Show the plot
plt.show()
