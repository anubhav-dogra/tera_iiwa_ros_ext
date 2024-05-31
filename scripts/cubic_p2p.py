import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class CubicTrajectoryGenerator:
    def __init__(self):
        rospy.init_node('cubic_p2p')
        self.sub=rospy.Subscriber("/iiwa/joint_states",JointState,self.callback,queue_size=10)
        self.pub=rospy.Publisher("/iiwa/PositionController/command", Float64MultiArray, queue_size=10)
        self.rate_value = 100
        self.rate = rospy.Rate(self.rate_value)  # 10 Hz
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
        self.curr_positions = None
        self.generate_and_publish_trajectory()


    # Joint cubic trajectory generator
    def callback(self,data):
        self.curr_positions = data.position
        # print(self.joints)

    def generate_cubic_trajectory(self, qi, qf, tf, t_step):
        a0 = qi
        a1 = 0
        a2 = 3*(qf - qi)/(tf**2)
        a3 = -2*(qf - qi)/(tf**3)

        t = np.arange(0, tf, t_step)
        q = np.zeros(len(t))

        for i, ti in enumerate(t):
            q[i] = a0 + a1*ti + a2*ti**2 + a3*ti**3 

        return t, q


    def generate_and_publish_trajectory(self):
        # qi = np.array([0,0,0,0,0,0,0])
        if self.curr_positions is None:
            rospy.sleep(0.5)
        qi = self.curr_positions
        q1 = np.array([-0.79, -1.33, -1.7751, -1.6761, -1.3454, -1.7916, -0.833])
        q2 = np.array([-0.77, -1.478, -1.8924, -1.494, -1.442, -1.874, -0.695])
        q3 = np.array([-0.6656, -1.2372, -1.697, -1.774, -1.271, -1.756, -1.052])
        q4 = np.array([-0.733, -1.6307, -1.9705, -1.2560, -1.491, -1.944, -0.5382])
        q5 = np.array([-1.0812, -1.3499, -1.7757, -1.5639, -1.3406, -1.7712, -0.452])
        # qf = np.array([0,0,0,0,0,0,0])
        # qf = np.array(np.random.uniform(-2, 2, 7))
        tf = 5
        t_step = 1/self.rate_value
        trajectories = []
        for i in range(len(qi)):
            t, q = self.generate_cubic_trajectory(qi[i], q1[i], tf, t_step)
            trajectories.append(q)
        for i in range(len(qi)):
                plt.plot(t, trajectories[i], label=self.joint_names[i])
                plt.legend()
                plt.xlabel("Time (s)")
                plt.ylabel("Joint Position (rad)")
                plt.grid(True)
                plt.show()

        for step in range(len(t)):
            msg = Float64MultiArray()
            msg.data = [trajectories[0][step], trajectories[1][step], trajectories[2][step], trajectories[3][step], trajectories[4][step], trajectories[5][step], trajectories[6][step]]
            self.pub.publish(msg)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        CubicTrajectoryGenerator()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass