#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
from cartesian_impedance_controller.msg import ControllerConfig
import numpy as np
from iiwa_tools.srv import GetJacobian, GetFK, GetGravity, GetMassMatrix
from scipy.optimize import minimize, Bounds
from urdf_parser_py.urdf import URDF
import xml.dom.minidom
from pyquaternion import Quaternion

class OptimizeWrench():
    def __init__(self) -> None:
        rospy.init_node('wrench_optimizer',anonymous=True)
        # Define subscribers and publishers
        self.g_pub = rospy.Publisher('gravity_compensation', Float64MultiArray, queue_size=10)
        self.wrench_pub = rospy.Publisher('cartesian_wrench_test', WrenchStamped, queue_size=10)
        self.pub_controller_config = rospy.Publisher('/iiwa/CartesianImpedance_trajectory_controller/set_config', ControllerConfig, queue_size=10)
        self.joint_state_subscriber = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback,queue_size=10)
        self.ee_state_subscriber = rospy.Subscriber('/tool_link_ee_pose', PoseStamped, self.ee_state_callback,queue_size=10)
        self.tau_commanded_sub = rospy.Subscriber('/iiwa/CartesianImpedance_trajectory_controller/commanded_torques', Float64MultiArray, self.tau_commanded_callback,queue_size=10)
        self.robot_description = rospy.get_param('/robot_description')
        # self.upper_joint_limits, self.lower_joint_limits = self.get_joint_limits(self.robot_description)
        self.curr_joint_state = None
        self.curr_vel_state = None
        pass

    def tau_commanded_callback(self,msg):
        self.tau_commanded = msg.data
        # wrenches = self.get_wrench(self.curr_joint_state,self.tau_commanded)
        # print(wrenches)
        # print(self.tau_commanded)

    def wrench_error_estimator(self,q):
        cov_mat = np.linalg.inv(np.identity(7))
        d = np.array([0, 0, 0, 0, 0, -1])
        J = self.cal_Jacobian(q)
        wrench_uncertainity = np.linalg.inv(np.dot(J, np.dot(cov_mat, J.T)))
        u = np.dot(d.T,np.dot(wrench_uncertainity,d))
        return u


    def joint_state_callback(self,data):
        if data is None:
            rospy.sleep(0.1)
        self.curr_joint_state = data.position[0:7]
        self.curr_vel_state = data.velocity[0:7]
        self.curr_effort_state = data.effort[0:7]
        # u=self.wrench_error_estimator(self.curr_joint_state)
        # print("wrench_error_estimator",u)
        wrenches = self.get_wrench(self.curr_joint_state,self.curr_effort_state)
        # print("wrenches",wrenches) # its in base frame!!
        C_wrench = WrenchStamped()
        C_wrench.header.frame_id = "world"
        C_wrench.header.stamp = rospy.Time.now()
        C_wrench.wrench.force.x = wrenches[3]
        C_wrench.wrench.force.y = wrenches[4]
        C_wrench.wrench.force.z = wrenches[5]
        C_wrench.wrench.torque.x = wrenches[0]
        C_wrench.wrench.torque.y = wrenches[1]
        C_wrench.wrench.torque.z = wrenches[2]
        self.wrench_pub.publish(C_wrench)

        # print(self.curr_effort_state)
        # Mass = self.get_mass_matrix(self.curr_joint_state)
        # Gravity = self.get_gravity_compensation(self.curr_joint_state,self.curr_vel_state)
        # g_msg = Float64MultiArray()
        # g_msg.data = Gravity
        # self.g_pub.publish(g_msg)
        # print("Gravity", Gravity)
        # print("diff",np.subtract(self.curr_effort_state,Gravity))

        

    def get_mass_matrix(self,q):
        mass_service = '/iiwa/iiwa_mass_server'
        rospy.wait_for_service(mass_service)
        good = False
        while not good:
            try:
                get_mass = rospy.ServiceProxy(mass_service, GetMassMatrix)
                mass_resp = get_mass(joint_angles=q)
                mass_out = mass_resp.mass_matrix
                dim = mass_out.layout.dim
                data_offset = mass_out.layout.data_offset
                sizes = [d.size for d in dim]
                data = mass_out.data
                Mass_Matrix = np.array(data[data_offset:]).reshape(sizes)
                print("Mass Matrix")
                print(Mass_Matrix)
                good = True
            except rospy.ServiceException as e:
                 print("Service call failed: %s" %e)

    def get_gravity_compensation(self,q, qdot):
        gravity_service = '/iiwa/iiwa_gravity_server'
        rospy.wait_for_service(gravity_service)
        good = False
        while not good:
            try:
                get_gravity = rospy.ServiceProxy(gravity_service,GetGravity)
                g_resp = get_gravity(joint_angles=q, joint_velocities=qdot,joint_torques=[10,10,10,10,10,10,10],gravity=[0,0,-9.80665])
                g_out = g_resp.compensation_torques
                
                good = True
            except rospy.ServiceException as e:
                 print("Service call failed: %s" %e)
        return g_out

    def cal_Jacobian(self, q):
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
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        return J
    
    def get_wrench(self, q, tau_ext):
        J = self.cal_Jacobian(q)
        J_t_Inv = np.linalg.pinv(J.T)
        wrenches = np.dot(J_t_Inv,tau_ext)
        return wrenches




if __name__ == '__main__':

    try:
        optimizer_node = OptimizeWrench()
        rospy.spin()
        # optimizer_node.run()
    except rospy.ROSInterruptException:
        pass