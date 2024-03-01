import rospy
from sensor_msgs.msg import JointState
from cartesian_impedance_controller.msg import ControllerConfig
import numpy as np
from iiwa_tools.srv import GetJacobian, GetFK
from scipy.optimize import minimize, Bounds
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from urdf_parser_py.urdf import URDF
import xml.dom.minidom
from pyquaternion import Quaternion
from geometry_msgs.msg import Pose, PoseStamped

def get_joint_limits(robot_description):
    robot = URDF.from_xml_string(robot_description)
    u_joint_limit = []
    l_joint_limit = []
    # joint_limits = {}
    # for joint in robot.joints:
    #     if joint.limit:
    #         joint_limits[joint.name] = {
    #             'lower': joint.limit.lower,
    #             'upper': joint.limit.upper
    #         }
    # print(joint_limits)
    for joint in robot.joints:
        if joint.limit:
            u_joint_limit.append(joint.limit.upper)
            l_joint_limit.append(joint.limit.lower)

    return u_joint_limit, l_joint_limit
    

def call_FK(joint_position_):
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
            curr_joints.data = joint_position_
            resp = get_fk(joints = curr_joints)
            pose_out = resp.poses[0]
            # print(pose_out)
            good = True
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
    return pose_out

def cal_Jacobian(q):
    jac_service = "/iiwa/iiwa_jacobian_server"
    rospy.wait_for_service(jac_service)
    good = False
    while not good:
        try:
            # Create a proxy for the service
            get_jac = rospy.ServiceProxy(jac_service, GetJacobian)
            jac_resp = get_jac(joint_angles = q, joint_velocities=joint_velocities)        
            jac_out = jac_resp.jacobian
            jac_array = jac_out.data
            jac_layout = jac_out.layout
            jac_dim_sizes = [dim.size for dim in jac_layout.dim]
            J = np.array(jac_array).reshape(jac_dim_sizes)
            # print(J)
            good = True
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
    return J

def cal_manipulability(joint_position_):
    J = cal_Jacobian(joint_position_)
    
    # Assuming 'J' is the Jacobian matrix with shape (6, 7)
    # Calculate the product of J and its transpose J^T
    JJT = np.dot(J, J.T)
    # Alternatively, you can use JJT = J @ J.T

    manipulability = np.sqrt(np.linalg.det(JJT))
    return -manipulability
    # print(manipulability)


def cons_eq(joint_position_):
    # global desired_pose
    pose_now = call_FK(joint_position)

    if desired_pose is None:
        # if isinstance(self.desired_pose, Pose):  # Check if it's a Pose object
            pose_now = desired_pose_  # Copy the reference
            print("Im HERE **********************")
    else:
        pose_now = Pose()  # Create a new Pose object if necessary
        pose_now.position.x = desired_pose.pose.position.x 
        pose_now.position.y = desired_pose.pose.position.y
        pose_now.position.z = desired_pose.pose.position.z
        pose_now.orientation.x = desired_pose.pose.orientation.x
        pose_now.orientation.y = desired_pose.pose.orientation.y
        pose_now.orientation.z = desired_pose.pose.orientation.z
        pose_now.orientation.w = desired_pose.pose.orientation.w

    pose_opti = call_FK(joint_position_)
    # pos_error = [pose_now.position.x-pose_opti.position.x,
    #                 pose_now.position.y-pose_opti.position.y,
    #                 pose_now.position.z-pose_opti.position.z]
    

    o_now = np.array([pose_now.orientation.w,pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z])
    o_opti = np.array([pose_opti.orientation.w,pose_opti.orientation.x,pose_opti.orientation.y,pose_opti.orientation.z])
    if np.dot(o_now,o_opti) < 0:
        o_opti = -o_opti

    o_now_q = Quaternion(o_now)
    o_opti_q = Quaternion(o_opti)
    q_error = o_opti_q*(o_now_q.inverse)
    ax = q_error.axis
    ang = q_error.angle
    ax_ang = ax*ang
    # o_error = [ax_ang[0], ax_ang[1], ax_ang[2]]

    pose_error = [pose_now.position.x-pose_opti.position.x,
                    pose_now.position.y-pose_opti.position.y,
                    pose_now.position.z-pose_opti.position.z,
                    ax_ang[0],
                    ax_ang[1],
                    ax_ang[2]]
    # print("pose_error",pose_error)
    return pose_error


def callback_joints(data):
    global joint_velocities, joint_position, desired_pose_, desired_pose
    joint_position = data.position[0:7]
    joint_velocities = data.velocity[0:7]
    # joint_velocities = [0,0,0,0,0,0,0]
    # m = cal_manipulability(joint_position)
    # print(m)
    if desired_pose is None:
        desired_pose_ = call_FK(joint_position)


    # Optimize manipulability
    initial_guess = joint_position
    # print("initial_guess",initial_guess)
    bounds = Bounds(lower_joint_limits, upper_joint_limits)
    cons = ({'type': 'eq', 'fun': cons_eq})
    optimization_options = {'disp': True}  # Display optimization progress
    opt_result = minimize(cal_manipulability,initial_guess,method="SLSQP",bounds=bounds, constraints=cons,options=optimization_options)
    # print("optimization_message",opt_result.message)
    optimal_joint_configuration = opt_result.x
    # print("optimal_joint_configuration", optimal_joint_configuration)
    ## to verify if FK is satisfied
    # print("pose_optimzied", call_FK(optimal_joint_configuration))

    # check nullspace command sending. 
    # factor = -0.01
    # joint_values = [joint_position[0]+factor,
    #                 joint_position[1]+factor,
    #                 joint_position[2]+factor,
    #                 joint_position[3],
    #                 joint_position[4]+factor,
    #                 joint_position[5]+factor,
    #                 joint_position[6]+factor]
    
    pose_error = cons_eq(optimal_joint_configuration)
    max_abs_value = np.max(np.abs(pose_error))
    print(max_abs_value)
    if max_abs_value < 0.0001:
        controller_config = set_controller_config(optimal_joint_configuration)
        # print("controller_config", controller_config.q_d_nullspace)
        pub_controller_config.publish(controller_config)
    else:
        print("Error is large ! ")
        pass
    
def set_controller_config(joint_values):
    msg = ControllerConfig()

    msg.cartesian_stiffness.force.x = 1000
    msg.cartesian_stiffness.force.y = 1000
    msg.cartesian_stiffness.force.z = 800
    msg.cartesian_stiffness.torque.x = 30
    msg.cartesian_stiffness.torque.y = 30
    msg.cartesian_stiffness.torque.z = 30

    # Negative values mean that the default damping values apply --> 2* sqrt(stiffness)
    msg.cartesian_damping.force.x =  2*np.sqrt(msg.cartesian_stiffness.force.x)
    msg.cartesian_damping.force.y =  2*np.sqrt(msg.cartesian_stiffness.force.y)
    msg.cartesian_damping.force.z =  2*np.sqrt(msg.cartesian_stiffness.force.z)
    msg.cartesian_damping.torque.x = 2*np.sqrt(msg.cartesian_stiffness.torque.x)
    msg.cartesian_damping.torque.y = 2*np.sqrt(msg.cartesian_stiffness.torque.y)
    msg.cartesian_damping.torque.z = 2*np.sqrt(msg.cartesian_stiffness.torque.z)


    if reset_flag is False:
        msg.q_d_nullspace.append(joint_values[0])
        msg.q_d_nullspace.append(joint_values[1])
        msg.q_d_nullspace.append(joint_values[2])
        msg.q_d_nullspace.append(joint_values[3])
        msg.q_d_nullspace.append(joint_values[4])
        msg.q_d_nullspace.append(joint_values[5])
        msg.q_d_nullspace.append(joint_values[6])
            
        msg.nullspace_stiffness = 5
    else:
        msg.q_d_nullspace = []
        msg.nullspace_stiffness = 0
    msg.nullspace_damping = 2*(np.sqrt(msg.nullspace_stiffness))
    return msg
    
def reference_pose_callback(pose_reference):
    global desired_pose
    desired_pose = pose_reference

def reset_nullspace():
    global joint_position, reset_flag
    reset_flag = True
    controller_config = set_controller_config(joint_position)
    pub_controller_config.publish(controller_config)



if __name__=='__main__':
    global upper_joint_limits, lower_joint_limits, reset_flag
    rospy.init_node("nullspace_controller")
    pub_controller_config = rospy.Publisher('/iiwa/CartesianImpedance_trajectory_controller/set_config', ControllerConfig, queue_size=10)
    reset_flag = False
    ## if required in loop
    robot_description = rospy.get_param('/robot_description')
    upper_joint_limits, lower_joint_limits = get_joint_limits(robot_description)
    reference_pose_subscriber = rospy.Subscriber('/iiwa/CartesianImpedance_trajectory_controller/reference_pose', PoseStamped, reference_pose_callback, queue_size=1)
    joint_sub = rospy.Subscriber('/iiwa/joint_states',JointState,callback_joints,queue_size=1)
    rospy.on_shutdown(reset_nullspace)
    rospy.spin()
    

    ## this run only once. 
    # msg_ = rospy.wait_for_message('/iiwa/joint_states',JointState)
    # callback_joints(msg_)


    # positions = msg_.position[0:7]
    # velocities = msg_.velocity[0:7]
    # factor = 0.1
    # print("joint_positions_now", msg_.position[0:7])
    # joint_values = [positions[0]+factor,
    #                 positions[1]+factor,
    #                 positions[2]+factor,
    #                 positions[3],
    #                 positions[4]+factor,
    #                 positions[5]+factor,
    #                 positions[6]+factor]
    # # joint_values = [0.0003,0.0005,0.0005,0,0,0,0]
    # controller_config = set_controller_config(joint_values)
    # print("controller_config", controller_config)
    # rospy.sleep(2)
    # pub_controller_config.publish(controller_config)
    # print("controller_config", controller_config)
    
    
    
    