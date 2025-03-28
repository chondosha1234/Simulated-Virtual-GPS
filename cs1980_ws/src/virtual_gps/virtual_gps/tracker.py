import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from matplotlib.animation import FuncAnimation

# getting actual locations
file_path = './measurements/actual_data.json'

with open(file_path, 'r') as json_file:
    actual_data = json.load(json_file)

# getting calculated locations
file_path = './measurements/calc_data.json'

with open(file_path, 'r') as json_file:
    calc_data = json.load(json_file)

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot()

# Separate the x, y, and z coordinates
actual_x_vals = [point["x"] for point in actual_data]
actual_y_vals = [point["y"] for point in actual_data]

calc_x_vals = [point["x"] for point in calc_data]
calc_y_vals = [point["y"] for point in calc_data]


# create a figure
fig, ax = plt.subplots()

# Plot calculated location first
scat1 = ax.scatter(calc_x_vals, calc_y_vals, color='red', label='Calculated Location')

# Add labels and title
ax.set_xlabel('x Position')
ax.set_ylabel('y Position')
ax.set_title('Actual vs. Calculated Locations')

# Show the legend
ax.legend()

# Function to update the plot with the actual location
def update(frame):
    if frame == 1:
        ax.scatter(actual_x_vals, actual_y_vals, marker='x', color='green', label='Actual Location')
        ax.legend()

# Create the animation
ani = FuncAnimation(fig, update, frames=[0, 1], interval=2000, repeat=False)

# Show the plot
plt.show()





# # Connect the points with a line
# ax.scatter(actual_x_vals, actual_y_vals, color='g', label='Actual Location')
# ax.scatter(calc_x_vals, calc_y_vals, color='r', label='Calculated Location')

# # Label the axes
# ax.set_xlabel('x Position')
# ax.set_ylabel('y Position')

# # Set the title of the plot
# ax.set_title('Actual vs. Calculated Locations')

# # Show legend
# ax.legend()

# # Display the plot
# plt.show()
