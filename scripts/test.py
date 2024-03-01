import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from iiwa_tools.srv import GetFK
import numpy as np
import matplotlib.pyplot as plt



# Specify the filename here
filename = "/home/terabotics/stuff_ws/src/tera_iiwa_ros/scripts/manipulability_test_ee2.txt"
filename1 = "/home/terabotics/stuff_ws/src/tera_iiwa_ros/scripts/wrench_uncertainity_test_ee2.txt"
# Read the data from the text file
with open(filename, "r") as f:
    data = [float(line.strip()) for line in f]

# Create the plot
plt.plot(data,linewidth=2)
# Enable grid and box
# plt.grid(True)
plt.box(True)

# Set font size
plt.rcParams.update({'font.size': 17})
# Add labels and title to the plot (optional)
plt.xlabel("Joint state subscription message")
plt.ylabel("Manipulability")
# plt.title(f"Data from {filename}")

# Show the plot
plt.show()

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

exit()

Tf = np.array([[1, 2, 3],
               [4, 5, 6],
               [7, 8, 9]])

# Define the matrix T
T = np.block([
    [Tf[:3, :3], np.zeros((3, 3))],
    [np.zeros((3, 3)),Tf[:3, :3]]
])

print(T)
exit()
# def call_FK(joint_position_):
#     fk_service = '/iiwa/iiwa_fk_server'
#     rospy.wait_for_service(fk_service)
#     good = False
#     while not good:
#         try:
#             # Create a proxy for the service
#             get_fk = rospy.ServiceProxy(fk_service, GetFK)
#             curr_joints = Float64MultiArray()
#             curr_joints.layout = MultiArrayLayout()
#             curr_joints.layout.dim = [MultiArrayDimension(),MultiArrayDimension()]
#             curr_joints.layout.dim[0].size=1
#             curr_joints.layout.dim[1].size=7
#             curr_joints.data = joint_position_
#             resp = get_fk(joints = curr_joints)
#             pose_out = resp.poses[0]
#             print(pose_out)
#             good = True
#         except rospy.ServiceException as e:
#             print("Service call failed: %s" %e)
#     return pose_out
# # q = [-0.560753, -1.49692, 2.09722, 1.26435, 1.47737, 2.0944, -0.87233]
# q = [-0.753135, -1.69922, 1.722428, 1.56627, 1.722333, 1.95015, -0.69854]
# call_FK(q)