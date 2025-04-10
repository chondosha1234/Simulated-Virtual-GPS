import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# getting actual locations
file_path = './measurements/actual_data.json'

with open(file_path, 'r') as json_file:
    actual_data = json.load(json_file)

# getting calculated locations
file_path = './measurements/calc_data.json'

with open(file_path, 'r') as json_file:
    calc_data = json.load(json_file)

# Create a figure and a 3D axis
fig = plt.figure(figsize=(10,6))
ax = fig.add_subplot(111, projection='3d')

y_min, y_max = -2.0, 2.0

# Separate the x, y, and z coordinates
actual_x_vals = [point["x"] for point in actual_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]
actual_y_vals = [point["y"] for point in actual_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]
actual_z_vals = [point["z"] for point in actual_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]

calc_x_vals = [point["x"] for point in calc_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]
calc_y_vals = [point["y"] for point in calc_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]
calc_z_vals = [point["z"] for point in calc_data if y_min <= point["x"] <= y_max and y_min <= point["y"] <= y_max]

# Connect the points with a line
ax.scatter(actual_x_vals, actual_z_vals, actual_y_vals, color='g', label='Actual Location')
ax.scatter(calc_x_vals, calc_z_vals, calc_y_vals, color='r', label='Calculated Location')

# Label the axes
ax.set_xlabel('x Position')
ax.set_ylabel('y Position')
ax.set_zlabel('z Position')

# Set the title of the plot
ax.set_title('Actual vs. Calculated Locations')

# Show legend
ax.legend()

# Display the plot
plt.show()
