import rospy
import numpy as np
import matplotlib.pyplot as plt


#  NOT WORKING, NO CHECKS on VMAX or AMAX, THEREFORE GIVING WRONG ANSWERS


class TrapezoidalTrajectoryGenerator:
    def __init__(self):
        rospy.init_node('trapezoidal_trajectory_generator')
        # self.trajectory_pub = rospy.Publisher('/kuka_trajectory', JointTrajectory, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

        self.generate_and_publish_trajectory()


    def generate_and_publish_trajectory(self):
        qi = np.array([0,0,0,0,0,0,0])
        qf = np.array([-0.79, -1.33, -1.77, -1.67, -1.34, -1.79, -0.83])
        vmax = 1
        amax = 0.5
        t_step = 0.05
        durations = []

        # Calculate durations for each joint
        for i in range(len(qi)):
            dq = qf[i] - qi[i]
            tq = vmax / amax
            if dq == 0:
                durations.append(0)
            else:
                T = (abs(dq) / vmax) + tq
                durations.append(T)

        # Find the maximum duration
        max_duration = max(durations)
        t = np.arange(0, max_duration + t_step, t_step)

        # Generate synchronized trajectories
        trajectories = []
        for i in range(len(qi)):
            q = self.generate_trapezoidal_profile(qi[i], qf[i], vmax, amax, t)
            trajectories.append(q)        
        # plt.plot(t, trajectories[0], 'b-', t, trajectories[1], 'g-', t, trajectories[2], 'r-', t, trajectories[3], 'y-', t, trajectories[4], 'k-', t, trajectories[5], 'm-', t, trajectories[6], 'c-')
        for i in range(len(qi)):
            plt.plot(t, trajectories[i], label=self.joint_names[i])
            plt.legend()
            plt.xlabel("Time (s)")
            plt.ylabel("Joint Position (rad)")
            plt.grid(True)
            plt.show()


    def generate_trapezoidal_profile(self, qi, qf, vmax, amax, t):
        dq = qf - qi
        tq = vmax / amax
        if dq == 0:
            return np.full_like(t, qi)
        T = (abs(dq) / vmax) + tq
        t1 = tq
        t2 = T - tq
        q = np.zeros_like(t)

        for i, ti in enumerate(t):
            if ti <= t1:
                q[i] = qi + 0.5 * amax * ti**2 * np.sign(dq)
            elif ti <= t2:
                q[i] = qi + 0.5 * amax * t1**2 * np.sign(dq) + vmax * (ti - t1) * np.sign(dq)
            else:
                q[i] = qf - 0.5 * amax * (T - ti)**2 * np.sign(dq)

        return q

if __name__ == '__main__':
    try:
        TrapezoidalTrajectoryGenerator()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
# amax = 0.1
# check vmax
# if vmax  < abs(qf[1]-qi[1])/tf:
#     rospy.logerr("vmax is too small")
#     # end program
#     exit()
# elif vmax > 2*abs(qf[1]-qi[1])/tf:
#     rospy.logerr("vmax is too large")
#     exit()
# else:
#     rospy.loginfo("vmax is ok")


# t_c = (abs(qi[0])-abs(qf[0])+vmax*tf)/vmax
# # tf = abs(qf[0]-qi[0])/vmax + t_c
# amax = vmax/t_c
# t_step = 0.01

# # t_c = 0.5*tf - 0.5*np.sqrt((amax*tf**2 - 4*(qf[1]-qi[1]))/amax)

# t = np.arange(0, tf, t_step)
# q = np.zeros(len(t))
# for i in range(len(t)):
#     if t[i] < t_c:
#         q[i] = qi[0] + 0.5*amax*t[i]**2
#     elif t[i] < tf - t_c:
#         q[i] = qi[0] + amax*t_c*(t[i]-0.5*t_c)
#     else:
#         q[i] = qf[0]-0.5*amax*(tf-t[i])**2


# plt.plot(t, q)
# plt.show()


