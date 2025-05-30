import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from matplotlib.animation import FuncAnimation
import time

# getting actual locations
#file_path = './measurements/old_data/2_drone_raw_move_actual.json'
file_path = './measurements/actual_data.json'

with open(file_path, 'r') as json_file:
    actual_data = json.load(json_file)

# getting calculated locations
#file_path = './measurements/old_data/2_drone_kalman.json'
file_path = './measurements/kalman_data.json'

with open(file_path, 'r') as json_file:
    kalman_data = json.load(json_file)



y_min, y_max = -5.0, 5.0

# Filter and collect valid points from both datasets
actual_filtered = [
    point for point in actual_data
    if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max
]
kalman_filtered = [
    point for point in kalman_data
    if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max
]

# Scatter plot for actual positions
actual_x_vals = [p["x"] for p in actual_filtered]
actual_y_vals = [p["y"] for p in actual_filtered]
plt.scatter(actual_x_vals, actual_y_vals, color='green', label='Actual Locations', s=10, zorder=1)

# Scatter plot for calculated positions
kalman_x_vals = [p["x"] for p in kalman_filtered]
kalman_y_vals = [p["y"] for p in kalman_filtered]
plt.scatter(kalman_x_vals, kalman_y_vals, color='red', label='Calculated Locations', s=10, zorder=3)

# Draw lines between corresponding actual and calculated positions
for a, c in zip(actual_filtered, kalman_filtered):
    plt.plot([a["x"], c["x"]], [a["y"], c["y"]], color='gray', linewidth=0.5, zorder=0)

# Add labels and title
plt.xlabel('x Position')
plt.ylabel('y Position')
plt.title('Actual vs. Calculated Locations')
plt.legend()

plt.xlim(-3.0, 3.0)
plt.ylim(-3.0, 3.0)

# Show plot
plt.show()


"""
y_min, y_max = -5.0, 5.0

# Separate the x, y, and z coordinates
actual_x_vals = [point["x"] for point in actual_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]
actual_y_vals = [point["y"] for point in actual_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]

kalman_x_vals = [point["x"] for point in kalman_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]
kalman_y_vals = [point["y"] for point in kalman_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]

# Scatter plot for the actual location
plt.scatter(actual_x_vals, actual_y_vals, color='green', label='Actual Locations', s=10, zorder = 2)

# Scatter plot for the second dataset
plt.scatter(kalman_x_vals, kalman_y_vals, color='red', label='Calculated Locations', zorder=1)

# Add labels and title
plt.xlabel('x Position')
plt.ylabel('y Position')
plt.title('Actual vs. Calculated Locations')

# Add a legend to differentiate the two datasets
plt.legend()

# Show the plot
plt.show()
"""