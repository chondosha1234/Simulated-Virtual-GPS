import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from matplotlib.animation import FuncAnimation
import time

# getting actual locations
file_path = './measurments/actual_data.json'

with open(file_path, 'r') as json_file:
    actual_data = json.load(json_file)

# getting calculated locations
file_path = './measurements/calc_data.json'

with open(file_path, 'r') as json_file:
    calc_data = json.load(json_file)

y_min, y_max = -2.0, 2.0

# Separate the x, y, and z coordinates
actual_x_vals = [point["x"] for point in actual_data if y_min <= point["y"] <= y_max]
actual_y_vals = [point["y"] for point in actual_data if y_min <= point["y"] <= y_max]

calc_x_vals = [point["x"] for point in calc_data if y_min <= point["y"] <= y_max]
calc_y_vals = [point["y"] for point in calc_data if y_min <= point["y"] <= y_max]

# Scatter plot for the actual location
plt.scatter(actual_x_vals, actual_y_vals, color='green', label='Actual Locations', s=10, zorder = 2)

# Scatter plot for the second dataset
plt.scatter(calc_x_vals, calc_y_vals, color='red', label='Calculated Locations', zorder=1)

plt.yticks(range(-2,3,1))

# Add labels and title
plt.xlabel('x Position')
plt.ylabel('y Position')
plt.title('Actual vs. Calculated Locations')

# Add a legend to differentiate the two datasets
plt.legend()

# Show the plot
plt.show()
