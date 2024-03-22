import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf
import math
from iiwa_tools.srv import GetIK, GetFK
import numpy as np
from angles import shortest_angular_distance, shortest_angular_distance_with_limits

joints = [0, 0, 0, 0, 0, 0, 0]

def callback(data):
    global joints
    joints = data.position

rospy.init_node('test')

pub = rospy.Publisher('/iiwa/PositionController/command', Float64MultiArray, queue_size=10)
rospy.Subscriber("/iiwa/joint_states", JointState, callback)

cmd_msg = Float64MultiArray()

listener = tf.TransformListener()

target = [0., 0., 0., 0., 0., 0., 0.]

ik_service = '/iiwa/iiwa_ik_server'
fk_service = '/iiwa/iiwa_fk_server'

rospy.wait_for_service(ik_service)
rospy.wait_for_service(fk_service)

lower_limits = -np.array([170, 120, 170, 120, 170, 120 ,175])
lower_limits = lower_limits*np.pi/180.
upper_limits = np.array([170, 120, 170, 120, 170, 120 ,175])
upper_limits = upper_limits*np.pi/180.

# Fill with desired end effector transformation
T_desired = np.array([[0,1,0,-0.62],
             [1,0,0,0],
             [0,0,-1,0.25],
             [0,0,0,1]])
R_desired = T_desired.copy() # the rotation part
R_desired[0:3, 3] = [0, 0, 0]
rot_desired = tf.transformations.quaternion_from_matrix(R_desired)

# Query IK to go to that location
pose_msg = Pose()
pose_msg.position.x = T_desired[0, 3]
pose_msg.position.y = T_desired[1, 3]
pose_msg.position.z = T_desired[2, 3]
pose_msg.orientation.x = rot_desired[0]
pose_msg.orientation.y = rot_desired[1]
pose_msg.orientation.z = rot_desired[2]
pose_msg.orientation.w = rot_desired[3]
print(pose_msg)

solution = []
good = False
while not good:
    # call the service
    try:
        get_ik = rospy.ServiceProxy(ik_service, GetIK)
        seed = Float64MultiArray()
        seed.layout = MultiArrayLayout()
        seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        seed.layout.dim[0].size = 1
        seed.layout.dim[1].size = 7
        seed.data = [0., 1., 0., -1., 0., 1., 0.]

        resp1 = get_ik(poses=[pose_msg], seed_angles=seed)
        solution = resp1.joints.data
        good = True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Dummy check to see if solution is good
good = False
while not good:
    # call the FK service
    try:
        get_fk = rospy.ServiceProxy(fk_service, GetFK)
        seed = Float64MultiArray()
        seed.layout = MultiArrayLayout()
        seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        seed.layout.dim[0].size = 1
        seed.layout.dim[1].size = 7
        seed.data = solution
        resp1 = get_fk(joints=seed)
        sol_pose = resp1.poses[0]
        print('sol:', sol_pose)
        good = True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# print(solution)
target = solution

while not rospy.is_shutdown():
    jt = joints

    diff = []
    for i in range(len(target)):
        diff.append(shortest_angular_distance_with_limits(jt[i], target[i], lower_limits[i], upper_limits[i])[1])
    print(diff)
    th = 1e-3
    for i in range(len(diff)):
        d = 0.003
        if diff[i] > d:
            diff[i] = d
        elif diff[i] < -d:
            diff[i] = -d
        if np.abs(diff[i]) < th:
            diff[i] = 0

    joint_target = np.array(jt) + np.array(diff)#np.multiply(np.array(diff), dt)

    cmd_msg.data = [joint_target[0], joint_target[1], joint_target[2], joint_target[3], joint_target[4], joint_target[5], joint_target[6]]
    pub.publish(cmd_msg)
    rospy.sleep(0.005)

rospy.spin()