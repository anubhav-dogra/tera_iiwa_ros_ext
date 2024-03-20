import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from iiwa_tools.srv import GetFK
import numpy as np
import matplotlib.pyplot as plt



# Specify the filename here
filename = "/home/terabotics/stuff_ws/src/tera_iiwa_ros_ext/data_recorded/robot_test_first/manipulability_robot_test2.txt"
# filename1 = "/home/terabotics/stuff_ws/src/tera_iiwa_ros_ext/scripts/wrench_uncertainity_check.txt"
# Read the data from the text file
with open(filename, "r") as f:
    data = [float(line.strip()) for line in f]

# Create the plot
plt.plot(data,linewidth=2)
# Enable grid and box
# plt.grid(True)
plt.box(True)

# Set font size
plt.rcParams.update({'font.size': 20})
# Add labels and title to the plot (optional)
plt.xlabel("Joint state subscription message")
plt.ylabel("Manipulability")
# plt.title(f"Data from {filename}")

# Show the plot
plt.show()
exit()
with open(filename1, "r") as f1:
    data1 = [float(line.strip()) for line in f1]

# Create the plot
plt.plot(data1,linewidth=2)
# Enable grid and box
# plt.grid(True)
plt.box(True)

# Set font size
plt.rcParams.update({'font.size': 17})
# Add labels and title to the plot (optional)
plt.xlabel("Joint state subscription message")
plt.ylabel("Wrench Uncertainity")
# plt.title(f"Data from {filename}")

# Show the plot
plt.show()

