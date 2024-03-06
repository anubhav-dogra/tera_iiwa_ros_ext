#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
from cartesian_impedance_controller.msg import ControllerConfig
import numpy as np
from iiwa_tools.srv import GetJacobian, GetFK
from scipy.optimize import minimize, Bounds
from urdf_parser_py.urdf import URDF
import xml.dom.minidom
from pyquaternion import Quaternion

class OptimizerNode:
    def __init__(self,base_file_name):
        rospy.init_node('optimizer_node', anonymous=True)
        self.rate = rospy.Rate(4)
        # Define subscribers and publishers
        self.pub_controller_config = rospy.Publisher('/iiwa/CartesianImpedance_trajectory_controller/set_config', ControllerConfig, queue_size=10)
        self.joint_state_subscriber = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback,queue_size=1)
        # self.ee_state_subscriber = rospy.Subscriber('/cartesian_wrench_tool', PoseStamped, self.ee_state_callback,queue_size=1)
        self.robot_description = rospy.get_param('/robot_description')
        self.upper_joint_limits, self.lower_joint_limits = self.get_joint_limits(self.robot_description)
        
        # Initialize optimization parameters
        self.curr_joint_state = None
        self.curr_vel_state = None
        # self.desired_pose = PoseStamped()
        self.desired_pose = None  # Initialize desired_pose variable
        self.Tf = np.eye(4)  # Identity matrix of size 4x4
        
        # Subscribe to reference pose topic
        self.reference_pose_subscriber = rospy.Subscriber('/iiwa/CartesianImpedance_trajectory_controller/reference_pose', PoseStamped, self.reference_pose_callback, queue_size=1)

        # Run optimization
        # self.run_optimization()
        self.optimization_timer = rospy.Timer(rospy.Duration(0.5), self.run_optimization)

        self.reset_flag = False
        rospy.on_shutdown(self.reset_nullspace)
        self.base_file_name = base_file_name
        self.output_file_manip = open(f"manipulability_{self.base_file_name}.txt", 'a')
        # self.output_file_wrench_uc = open(f"wrench_uncertainity_{self.base_file_name}.txt", 'a')

    def reference_pose_callback(self, reference_pose_msg):
        # Update desired_pose variable when a message is received on the reference_pose topic
        self.desired_pose = reference_pose_msg

    def get_joint_limits(self, robot_description):
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

    def joint_state_callback(self, joint_state):
        if joint_state is None:
            rospy.sleep(0.1)
        self.curr_joint_state = joint_state.position[0:7]
        self.curr_vel_state = joint_state.velocity[0:7]
        m = self.cal_manipulability(self.curr_joint_state)
        # wu = self.wrench_error_estimator(self.curr_joint_state)
        self.output_file_manip.write(str(m) + '\n')  # Write data to file
        self.output_file_manip.flush()  # Flush buffer to ensure data is written immediately
        # self.output_file_wrench_uc.write(str(wu) + '\n')  # Write data to file
        # self.output_file_wrench_uc.flush()  # Flush buffer to ensure data is written immediately
        pass

    def call_jacobian(self,q):
        ## Get Jacobian from the jacobian server
        jac_service = "/iiwa/iiwa_jacobian_server"
        rospy.wait_for_service(jac_service)
        good = False
        while not good:
            try:
                # Create a proxy for the service
                get_jac = rospy.ServiceProxy(jac_service, GetJacobian)
                jac_resp = get_jac(joint_angles = q, joint_velocities=self.curr_vel_state)        
                jac_out = jac_resp.jacobian
                jac_array = jac_out.data
                jac_layout = jac_out.layout
                jac_dim_sizes = [dim.size for dim in jac_layout.dim]
                J = np.array(jac_array).reshape(jac_dim_sizes)
                # print(J)
                good = True
                return J
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        

    def cal_manipulability(self,q):
        
        J = self.call_jacobian(q)
        
        # Assuming 'J' is the Jacobian matrix with shape (6, 7)
        # Calculate the product of J and its transpose J^T
        JJT = np.dot(J, J.T)
        # Alternatively, you can use JJT = J @ J.T

        manipulability = np.sqrt(np.linalg.det(JJT))
        return manipulability
        # print(manipulability)
    
    def call_FK(self, q):
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
                curr_joints.data = q
                resp = get_fk(joints = curr_joints)
                pose_out = resp.poses[0]
                # print("CallFK", pose_out)
                good = True
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        return pose_out
    
    def set_controller_config(self,joint_values):
        msg = ControllerConfig()

        msg.cartesian_stiffness.force.x = 1000
        msg.cartesian_stiffness.force.y = 1000
        msg.cartesian_stiffness.force.z = 800
        msg.cartesian_stiffness.torque.x = 50
        msg.cartesian_stiffness.torque.y = 50
        msg.cartesian_stiffness.torque.z = 50

        # Negative values mean that the default damping values apply --> 2* sqrt(stiffness)
        msg.cartesian_damping_factors.force.x =  2*np.sqrt(msg.cartesian_stiffness.force.x)
        msg.cartesian_damping_factors.force.y =  2*np.sqrt(msg.cartesian_stiffness.force.y)
        msg.cartesian_damping_factors.force.z =  2*np.sqrt(msg.cartesian_stiffness.force.z)
        msg.cartesian_damping_factors.torque.x = 2*np.sqrt(msg.cartesian_stiffness.torque.x)
        msg.cartesian_damping_factors.torque.y = 2*np.sqrt(msg.cartesian_stiffness.torque.y)
        msg.cartesian_damping_factors.torque.z = 2*np.sqrt(msg.cartesian_stiffness.torque.z)

            
        if self.reset_flag is False:
            msg.q_d_nullspace.append(joint_values[0])
            msg.q_d_nullspace.append(joint_values[1])
            msg.q_d_nullspace.append(joint_values[2])
            msg.q_d_nullspace.append(joint_values[3])
            msg.q_d_nullspace.append(joint_values[4])
            msg.q_d_nullspace.append(joint_values[5])
            msg.q_d_nullspace.append(joint_values[6])
            msg.nullspace_stiffness = 5
        else:
            msg.q_d_nullspace=[]
            msg.nullspace_stiffness = 0
        msg.nullspace_damping_factor = 2*(np.sqrt(msg.nullspace_stiffness))
        return msg
    

    def wrench_error_estimator(self,q):
        cov_mat = np.linalg.inv(np.identity(7))
        # d = np.array([0, 0, 0, 0, 0, 1])
        d = np.zeros(6)
        d[:3] = np.zeros(3)
        d[3:] = self.Tf[:3, 2]
        # print(d)
        J = self.call_jacobian(q)
        Tf_J = np.block([
                        [self.Tf[:3, :3],np.zeros((3,3))],
                        [np.zeros((3,3)),self.Tf[:3, :3]]
                        ])
        # print(J)
        J_ee = np.matmul(np.linalg.inv(Tf_J),J) # J in end-effector frame
        # print(J_ee)
        wrench_uncertainity = np.linalg.inv(np.dot(J_ee, np.dot(cov_mat, J_ee.T)))
        u = np.dot(d.T,np.dot(wrench_uncertainity,d))
        return u


 
    def cons_eq(self, q):
       
        # pose_now = self.call_FK(self.curr_joint_state)
        if self.desired_pose is None:
        # if isinstance(self.desired_pose, Pose):  # Check if it's a Pose object
            pose_now = self.desired_pose_  # Copy the reference
            print("Im taking FK **********************")
        else:
            pose_now = Pose()  # Create a new Pose object if necessary
            pose_now.position.x = self.desired_pose.pose.position.x 
            pose_now.position.y = self.desired_pose.pose.position.y
            pose_now.position.z = self.desired_pose.pose.position.z
            pose_now.orientation.x = self.desired_pose.pose.orientation.x
            pose_now.orientation.y = self.desired_pose.pose.orientation.y
            pose_now.orientation.z = self.desired_pose.pose.orientation.z
            pose_now.orientation.w = self.desired_pose.pose.orientation.w

        pose_opti = self.call_FK(q)

         # pos_error = [pose_now.position.x-pose_opti.position.x,
        #                 pose_now.position.y-pose_opti.position.y,
        #                 pose_now.position.z-pose_opti.position.z]
        

        o_now = np.array([pose_now.orientation.w,pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z])
        o_opti_ = np.array([pose_opti.orientation.w,pose_opti.orientation.x,pose_opti.orientation.y,pose_opti.orientation.z])
        if np.dot(o_now,o_opti_) < 0:
            o_opti = -o_opti_
        else:
            o_opti = o_opti_

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
        # print("pose_error in constraints",pose_error)
        rot_now = o_now_q.rotation_matrix

        # calculate Tf to transform things to eef
        self.Tf[:3, :3] = rot_now  # Assign orientation to the top-left 3x3 block
        self.Tf[:3, 3] = [pose_now.position.x,pose_now.position.y,pose_now.position.z]  # Assign position to the rightmost column of the top 3x3 block
        # print(self.Tf)
        return pose_error

    def objective_function(self, q):
        manipulability = self.cal_manipulability(q)
        return -manipulability  # Maximization problem, so negate manipulability
        # wrench_unc = self.wrench_error_estimator(q)
        # return (0.5*wrench_unc -0.5*manipulability)

    def run_optimization(self,event):
    # def run_optimization(self):
        # Define optimization parameters and constraints
        if self.curr_joint_state is None:
            rospy.sleep(0.1)
        initial_guess = self.curr_joint_state # Initial guess for joint configuration
        # print(initial_guess)

        # If desired_pose is not available from the /reference_pose topic, set it from call_FK
        if self.desired_pose is None:
            self.desired_pose_ = self.call_FK(self.curr_joint_state)


        bounds = Bounds(self.lower_joint_limits, self.upper_joint_limits)
        constraints_ = ({'type': 'eq', 'fun': self.cons_eq})  # Define constraints if any
        
        # Define optimization options
        optimization_options = {'disp': True}  # Display optimization progress
        
        # Run optimization
        result = minimize(self.objective_function, initial_guess, method="SLSQP",bounds=bounds, constraints=constraints_, options=optimization_options)
        
        # Extract optimal joint configuration
        optimal_joint_configuration = result.x
        # print("optimal_joint_configuration",optimal_joint_configuration)
        # print("Check Pose", self.call_FK(optimal_joint_configuration))
        pose_error = self.cons_eq(optimal_joint_configuration)
        max_abs_value = np.max(np.abs(pose_error))
        # print(max_abs_value)
        if max_abs_value < 0.00001:
            controller_config = self.set_controller_config(optimal_joint_configuration)
            # print("controller_config", controller_config.q_d_nullspace)
            self.pub_controller_config.publish(controller_config)
        else:
            print("Error is large ! ")
            pass

    def run(self):
        rospy.spin()
        # self.rate.sleep()

    def reset_nullspace(self):
        self.reset_flag = True
        controller_config = self.set_controller_config(np.zeros_like(self.curr_joint_state))
        self.pub_controller_config.publish(controller_config)
        self.output_file_manip.close()
        # self.output_file_wrench_uc.close()
        
        

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python script.py <base_file_name>")
        sys.exit(1)

    base_file_name = sys.argv[1]
    try:
        optimizer_node = OptimizerNode(base_file_name)
        rospy.spin()
        # optimizer_node.run()
    except rospy.ROSInterruptException:
        pass
