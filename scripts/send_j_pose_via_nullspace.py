
import rospy
import numpy as np
from cartesian_impedance_controller.msg import ControllerConfig


class SendJointPoseViaNullspace:
    def __init__(self):
          
        rospy.init_node('send_j_pose_via_nullspace', anonymous=True)
        self.pub_controller_config = rospy.Publisher('/iiwa/CartesianImpedance_trajectory_controller/set_config', ControllerConfig, queue_size=10)
        self.reset_flag = False
        self.joint_values = np.array([-0.79, -1.33, -1.77, -1.67, -1.34, -1.79, -0.833])
        # self.set_controller_config(self.joint_values)
        self.run()
        rospy.on_shutdown(self.reset_nullspace)
        
        

    def set_controller_config(self,joint_values):
            msg = ControllerConfig()

            msg.cartesian_stiffness.force.x = 400
            msg.cartesian_stiffness.force.y = 400
            msg.cartesian_stiffness.force.z = 400
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
            msg.nullspace_damping = 2*(np.sqrt(msg.nullspace_stiffness))
            return msg
    
    def run(self):
        while not rospy.is_shutdown():
            self.pub_controller_config.publish(self.set_controller_config(self.joint_values))

    def reset_nullspace(self):
            self.reset_flag = True
            controller_config = self.set_controller_config(np.zeros_like(self.joint_values))
            self.pub_controller_config.publish(controller_config)




if __name__ == '__main__':

    try:
        send_j_pose_node = SendJointPoseViaNullspace()
    except rospy.ROSInterruptException:
        pass