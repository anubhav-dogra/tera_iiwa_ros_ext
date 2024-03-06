import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from iiwa_tools.srv import GetFK
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from cartesian_impedance_controller.msg import ControllerConfig
import numpy as np

def set_controller_config_loose():
    msg = ControllerConfig()

    msg.cartesian_stiffness.force.x = 0
    msg.cartesian_stiffness.force.y = 0
    msg.cartesian_stiffness.force.z = 0
    msg.cartesian_stiffness.torque.x = 0
    msg.cartesian_stiffness.torque.y = 0
    msg.cartesian_stiffness.torque.z = 0

    # Negative values mean that the default damping values apply --> 2* sqrt(stiffness)
    msg.cartesian_damping_factors.force.x =  2*np.sqrt(msg.cartesian_stiffness.force.x)
    msg.cartesian_damping_factors.force.y =  2*np.sqrt(msg.cartesian_stiffness.force.y)
    msg.cartesian_damping_factors.force.z =  2*np.sqrt(msg.cartesian_stiffness.force.z)
    msg.cartesian_damping_factors.torque.x = 2*np.sqrt(msg.cartesian_stiffness.torque.x)
    msg.cartesian_damping_factors.torque.y = 2*np.sqrt(msg.cartesian_stiffness.torque.y)
    msg.cartesian_damping_factors.torque.z = 2*np.sqrt(msg.cartesian_stiffness.torque.z)

        
    msg.q_d_nullspace=[]
    msg.nullspace_stiffness = 0
    msg.nullspace_damping_factor = 2*(np.sqrt(msg.nullspace_stiffness))
    return msg

def set_controller_config_hard():
    msg = ControllerConfig()

    msg.cartesian_stiffness.force.x = 1000
    msg.cartesian_stiffness.force.y = 1000
    msg.cartesian_stiffness.force.z = 800
    msg.cartesian_stiffness.torque.x = 30
    msg.cartesian_stiffness.torque.y = 30
    msg.cartesian_stiffness.torque.z = 30

    # Negative values mean that the default damping values apply --> 2* sqrt(stiffness)
    msg.cartesian_damping_factors.force.x =  2*np.sqrt(msg.cartesian_stiffness.force.x)
    msg.cartesian_damping_factors.force.y =  2*np.sqrt(msg.cartesian_stiffness.force.y)
    msg.cartesian_damping_factors.force.z =  2*np.sqrt(msg.cartesian_stiffness.force.z)
    msg.cartesian_damping_factors.torque.x = 2*np.sqrt(msg.cartesian_stiffness.torque.x)
    msg.cartesian_damping_factors.torque.y = 2*np.sqrt(msg.cartesian_stiffness.torque.y)
    msg.cartesian_damping_factors.torque.z = 2*np.sqrt(msg.cartesian_stiffness.torque.z)

        
    msg.q_d_nullspace=[]
    msg.nullspace_stiffness = 0
    msg.nullspace_damping_factor = 2*(np.sqrt(msg.nullspace_stiffness))
    return msg

# def callback_joints(data):
#     curr_joint_pos = data.position[0:7]
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
#             curr_joints.data = curr_joint_pos
#             resp = get_fk(joints = curr_joints)
#             pose_out = resp.poses[0]
#             print(pose_out)
#             good = True
#         except rospy.ServiceException as e:
#             print("Service call failed: %s" %e)



if __name__=='__main__':
    rospy.init_node("hand_guiding",anonymous=True)
    pub_controller_config = rospy.Publisher('/iiwa/CartesianImpedance_trajectory_controller/set_config', ControllerConfig, queue_size=10)
    pose_pub = rospy.Publisher("/cartesian_trajectory_generator/new_goal",PoseStamped,queue_size=10)
    # joint_sub = rospy.Subscriber("/iiwa/joint_states",JointState,callback_joints)

    ## Change stiffness for hand-guiding. 
    controller_config = set_controller_config_loose()
    rospy.sleep(0.5)
    pub_controller_config.publish(controller_config)
    rospy.sleep(2)

    ## Do the Hand guiding
    input("Press Enter to set the current position as Reference Pose")

    # getting the current pose and setting it to the reference pose. 
    msg_ = rospy.wait_for_message('/iiwa/joint_states',JointState)
    curr_joint_pos = msg_.position[0:7]
    fk_service = '/iiwa/iiwa_fk_server'
    rospy.wait_for_service(fk_service)
    good = False
    while not good:
        try:
            # Create a proxy for the service
            get_fk = rospy.ServiceProxy(fk_service, GetFK)
            curr_joints = Float64MultiArray()
            curr_joints.layout = MultiArrayLayout()
            curr_joints.layout.dim = [MultiArrayDimension(),MultiArrayDimension()]
            curr_joints.layout.dim[0].size=1
            curr_joints.layout.dim[1].size=7
            curr_joints.data = curr_joint_pos
            resp = get_fk(joints = curr_joints)
            pose_out = resp.poses[0]
            print(pose_out)
            good = True
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    new_pose = PoseStamped()
    new_pose.header.frame_id = "world"
    new_pose.header.stamp = rospy.Time.now()
    new_pose.pose.position.x = pose_out.position.x
    new_pose.pose.position.y = pose_out.position.y
    new_pose.pose.position.z = pose_out.position.z
    new_pose.pose.orientation.x = pose_out.orientation.x
    new_pose.pose.orientation.y = pose_out.orientation.y
    new_pose.pose.orientation.z = pose_out.orientation.z
    new_pose.pose.orientation.w = pose_out.orientation.w
    print(new_pose)
    pose_pub.publish(new_pose)

    controller_config1 = set_controller_config_hard()
    rospy.sleep(1)
    pub_controller_config.publish(controller_config1)

    

    
    # rospy.spin()
    # rospy.sleep(0.1)